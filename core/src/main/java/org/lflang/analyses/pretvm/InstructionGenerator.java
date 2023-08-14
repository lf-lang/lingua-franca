package org.lflang.analyses.pretvm;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.IntStream;
import org.lflang.FileConfig;
import org.lflang.TargetConfig;
import org.lflang.TimeValue;
import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagEdge;
import org.lflang.analyses.dag.DagNode;
import org.lflang.analyses.dag.DagNode.dagNodeType;
import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;
import org.lflang.analyses.statespace.StateSpaceFragment;
import org.lflang.analyses.statespace.StateSpaceUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.TimerInstance;

/**
 * A generator that generates PRET VM programs from DAGs. It also acts as a linker that piece
 * together multiple PRET VM object files.
 *
 * @author Shaokai Lin
 */
public class InstructionGenerator {

  /** File configuration */
  FileConfig fileConfig;

  /** Target configuration */
  TargetConfig targetConfig;

  /** A list of reactor instances in the program */
  List<ReactorInstance> reactors;

  /** A list of reaction instances in the program */
  List<ReactionInstance> reactions;

  /** Number of workers */
  int workers;

  /** Constructor */
  public InstructionGenerator(
      FileConfig fileConfig,
      TargetConfig targetConfig,
      int workers,
      List<ReactorInstance> reactors,
      List<ReactionInstance> reactions) {
    this.fileConfig = fileConfig;
    this.targetConfig = targetConfig;
    this.workers = workers;
    this.reactors = reactors;
    this.reactions = reactions;
  }

  /** Traverse the DAG from head to tail using Khan's algorithm (topological sort). */
  public PretVmObjectFile generateInstructions(Dag dagParitioned, StateSpaceFragment fragment) {

    /** Instructions for all workers */
    List<List<Instruction>> instructions = new ArrayList<>();
    for (int i = 0; i < workers; i++) {
      instructions.add(new ArrayList<Instruction>());
    }

    // Initialize a queue and a map to hold the indegree of each node.
    Queue<DagNode> queue = new LinkedList<>();
    Map<DagNode, Integer> indegree = new HashMap<>();

    // Initialize a reaction index array to keep track of the latest counting
    // lock value for each worker.
    int[] countLockValues = new int[workers];

    // Debug
    int count = 0;

    // Initialize indegree of all nodes to be the size of their respective upstream node set.
    for (DagNode node : dagParitioned.dagNodes) {
      indegree.put(node, dagParitioned.dagEdgesRev.getOrDefault(node, new HashMap<>()).size());
      // Add the node with zero indegree to the queue.
      if (dagParitioned.dagEdgesRev.getOrDefault(node, new HashMap<>()).size() == 0) {
        queue.add(node);
      }
    }

    // The main loop for traversal using an iterative topological sort.
    while (!queue.isEmpty()) {
      // Dequeue a node.
      DagNode current = queue.poll();

      // Debug
      current.setDotDebugMsg("count: " + count++);

      // Get the upstream reaction nodes.
      List<DagNode> upstreamReactionNodes =
          dagParitioned.dagEdgesRev.getOrDefault(current, new HashMap<>()).keySet().stream()
              .filter(n -> n.nodeType == dagNodeType.REACTION)
              .toList();

      // Get the upstream sync nodes.
      List<DagNode> upstreamSyncNodes =
          dagParitioned.dagEdgesRev.getOrDefault(current, new HashMap<>()).keySet().stream()
              .filter(n -> n.nodeType == dagNodeType.SYNC)
              .toList();

      /* Generate instructions for the current node */
      if (current.nodeType == dagNodeType.REACTION) {
        // If the reaction depends on upstream reactions owned by other
        // workers, generate WU instructions to resolve the dependencies.
        // FIXME: Check if upstream reactions contain reactions owned by
        // other workers. If so, insert a WU with other workers'
        // countLockValues. The current implementation generates multiple WUs.
        for (DagNode n : upstreamReactionNodes) {
          int upstreamOwner = n.getWorker();
          if (upstreamOwner != current.getWorker()) {
            instructions
                .get(current.getWorker())
                .add(new InstructionWU(upstreamOwner, countLockValues[upstreamOwner]));
          }
        }

        // If the reaction depends on a single SYNC node,
        // advance to the LOGICAL time of the SYNC node first,
        // as well as delay until the PHYSICAL time indicated by the SYNC node.
        // Skip if it is the head node since this is done in SAC.
        // FIXME: Here we have an implicit assumption "logical time is
        // physical time." We need to find a way to relax this assumption.
        if (upstreamSyncNodes.size() == 1 && upstreamSyncNodes.get(0) != dagParitioned.head) {
          // Generate an ADV2 instruction.
          instructions
              .get(current.getWorker())
              .add(
                  new InstructionADV2(
                      current.getReaction().getParent(), upstreamSyncNodes.get(0).timeStep));
          // Generate a DU instruction if fast mode is off.
          if (!targetConfig.fastMode) {
            instructions
                .get(current.getWorker())
                .add(new InstructionDU(upstreamSyncNodes.get(0).timeStep));
          }
        } else if (upstreamSyncNodes.size() > 1)
          System.out.println("WARNING: More than one upstream SYNC nodes detected.");

        // If the reaction is triggered by startup, shutdown, or a timer,
        // generate an EXE instruction.
        // FIXME: Handle a reaction triggered by both timers and ports.
        ReactionInstance reaction = current.getReaction();
        if (reaction.triggers.stream()
            .anyMatch(
                trigger ->
                    (trigger.isStartup()
                        || trigger.isShutdown()
                        || trigger instanceof TimerInstance))) {
          instructions.get(current.getWorker()).add(new InstructionEXE(reaction));
        }
        // Otherwise, generate an EIT instruction.
        else {
          instructions.get(current.getWorker()).add(new InstructionEIT(reaction));
        }

        // Increment the counter of the worker.
        instructions
            .get(current.getWorker())
            .add(
                new InstructionADDI(
                    GlobalVarType.WORKER_COUNTER, GlobalVarType.WORKER_COUNTER, 1L));
        countLockValues[current.getWorker()]++;

      } else if (current.nodeType == dagNodeType.SYNC) {
        if (current == dagParitioned.tail) {
          // When the timeStep = TimeValue.MAX_VALUE in a SYNC node,
          // this means that the DAG is acyclic and can end without
          // real-time constraints, hence we do not genereate SAC,
          // DU, and ADDI.
          if (current.timeStep != TimeValue.MAX_VALUE) {
            for (var schedule : instructions) {
              // Add an SAC instruction.
              schedule.add(new InstructionSAC(current.timeStep));
              // Add a DU instruction if fast mode is off.
              if (!targetConfig.fastMode) schedule.add(new InstructionDU(current.timeStep));
              // Add an ADDI instruction.
              schedule.add(
                  new InstructionADDI(
                      GlobalVarType.WORKER_OFFSET,
                      GlobalVarType.WORKER_OFFSET,
                      current.timeStep.toNanoSeconds()));
            }
          }
        }
      }

      // Visit each downstream node.
      HashMap<DagNode, DagEdge> innerMap = dagParitioned.dagEdges.get(current);
      if (innerMap != null) {
        for (DagNode n : innerMap.keySet()) {
          // Decrease the indegree of the downstream node.
          int updatedIndegree = indegree.get(n) - 1;
          indegree.put(n, updatedIndegree);

          // If the downstream node has zero indegree now, add it to the queue.
          if (updatedIndegree == 0) {
            queue.add(n);
          }
        }
      }
    }

    // Check if all nodes are visited (i.e., indegree of all nodes are 0).
    // if (indegree.values().stream().anyMatch(deg -> deg != 0)) {
    //   // The graph has at least one cycle.
    //   throw new RuntimeException(
    //       "The graph has at least one cycle, thus cannot be topologically sorted.");
    // }

    return new PretVmObjectFile(instructions, fragment);
  }

  /** Generate C code from the instructions list. */
  public void generateCode(PretVmExecutable executable) {
    List<List<Instruction>> instructions = executable.getContent();

    // Instantiate a code builder.
    Path srcgen = fileConfig.getSrcGenPath();
    Path file = srcgen.resolve("static_schedule.c");
    CodeBuilder code = new CodeBuilder();

    // Generate a block comment.
    code.pr(
        String.join(
            "\n",
            "/**",
            " * An auto-generated schedule file for the STATIC scheduler.",
            " * ",
            " * reactor array:",
            " * " + this.reactors,
            " * ",
            " * reaction array:",
            " * " + this.reactions,
            " */"));

    // Header files
    code.pr(
        String.join(
            "\n",
            "#include <stdint.h>",
            "#include <stddef.h> // size_t",
            "#include \"tag.h\"",
            "#include \"core/threaded/scheduler_instructions.h\""));

    // Generate label macros.
    // Future FIXME: Make sure that label strings are formatted properly and are
    // unique, when the user is allowed to define custom labels. Currently,
    // all Phase enums are formatted properly.
    for (int i = 0; i < instructions.size(); i++) {
      var schedule = instructions.get(i);
      for (int j = 0; j < schedule.size(); j++) {
        if (schedule.get(j).hasLabel()) {
          code.pr("#define " + getWorkerLabelString(schedule.get(j).getLabel(), i) + " " + j);
        }
      }
    }

    // Extern variables.
    code.pr("extern instant_t " + getVarName(GlobalVarType.EXTERN_START_TIME, -1) + ";");

    // Generate variables.
    code.pr("volatile uint32_t " + getVarName(GlobalVarType.WORKER_COUNTER, workers) + " = {0};");
    code.pr("volatile uint64_t " + getVarName(GlobalVarType.WORKER_OFFSET, workers) + " = {0};");
    if (targetConfig.timeout != null)
      code.pr(
          "volatile uint64_t "
              + getVarName(GlobalVarType.GLOBAL_TIMEOUT, -1)
              + " = "
              + targetConfig.timeout.toNanoSeconds()
              + "LL"
              + ";");
    code.pr("const size_t num_counters = " + workers + ";");

    // Generate static schedules. Iterate over the workers (i.e., the size
    // of the instruction list).
    for (int i = 0; i < instructions.size(); i++) {
      var schedule = instructions.get(i);
      code.pr("const inst_t schedule_" + i + "[] = {");
      code.indent();

      for (int j = 0; j < schedule.size(); j++) {
        Instruction inst = schedule.get(j);

        // If there is a label attached to the instruction, generate a comment.
        if (inst.hasLabel()) code.pr("// " + getWorkerLabelString(inst.getLabel(), i) + ":");

        // Generate code based on opcode
        switch (inst.getOpcode()) {
          case ADDI:
            {
              InstructionADDI addi = (InstructionADDI) inst;
              String sourceVarName = "(uint64_t)&" + getVarName(addi.source, i);
              String targetVarName = "(uint64_t)&" + getVarName(addi.target, i);
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + "(Lock-free) increment "
                      + targetVarName
                      + " by adding "
                      + sourceVarName
                      + " and "
                      + addi.immediate
                      + "LL");
              code.pr(
                  "{.op="
                      + addi.getOpcode()
                      + ", "
                      + ".rs1="
                      + targetVarName
                      + ", "
                      + ".rs2="
                      + sourceVarName
                      + ", "
                      + ".rs3="
                      + addi.immediate
                      + "LL"
                      + "}"
                      + ",");
              break;
            }
          case ADV2:
            {
              ReactorInstance reactor = ((InstructionADV2) inst).reactor;
              TimeValue nextTime = ((InstructionADV2) inst).nextTime;
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + "(Lock-free) advance the logical time of "
                      + reactor
                      + " to "
                      + nextTime
                      + " wrt the variable offset");
              code.pr(
                  "{.op="
                      + inst.getOpcode()
                      + ", "
                      + ".rs1="
                      + reactors.indexOf(reactor)
                      + ", "
                      + ".rs2="
                      + "(uint64_t)&"
                      + getVarName(GlobalVarType.WORKER_OFFSET, i)
                      + ", "
                      + ".rs3="
                      + nextTime.toNanoSeconds()
                      + "LL"
                      + "}"
                      + ",");
              break;
            }
          case BEQ:
            {
              InstructionBEQ instBEQ = (InstructionBEQ) inst;
              String rs1Str = "(uint64_t)&" + getVarName(instBEQ.rs1, i);
              String rs2Str = "(uint64_t)&" + getVarName(instBEQ.rs2, i);
              Phase phase = instBEQ.phase;
              String labelString = getWorkerLabelString(phase, i);
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + "Branch to "
                      + labelString
                      + " if "
                      + rs1Str
                      + " = "
                      + rs2Str);
              code.pr(
                  "{.op="
                      + inst.getOpcode()
                      + ", "
                      + ".rs1="
                      + rs1Str
                      + ", "
                      + ".rs2="
                      + rs2Str
                      + ", "
                      + ".rs3="
                      + labelString
                      + "}"
                      + ",");
              break;
            }
          case BGE:
            {
              InstructionBGE instBGE = (InstructionBGE) inst;
              String rs1Str = "(uint64_t)&" + getVarName(instBGE.rs1, i);
              String rs2Str = "(uint64_t)&" + getVarName(instBGE.rs2, i);
              Phase phase = instBGE.phase;
              String labelString = getWorkerLabelString(phase, i);
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + "Branch to "
                      + labelString
                      + " if "
                      + rs1Str
                      + " >= "
                      + rs2Str);
              code.pr(
                  "{.op="
                      + inst.getOpcode()
                      + ", "
                      + ".rs1="
                      + rs1Str
                      + ", "
                      + ".rs2="
                      + rs2Str
                      + ", "
                      + ".rs3="
                      + labelString
                      + "}"
                      + ",");
              break;
            }
          case BIT:
            {
              // If timeout, jump to the EPILOGUE label.
              int stopIndex =
                  IntStream.range(0, schedule.size())
                      .filter(
                          k ->
                              (schedule.get(k).hasLabel()
                                  && schedule.get(k).getLabel().toString().equals("EPILOGUE")))
                      .findFirst()
                      .getAsInt();
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + "Branch, if timeout, to epilogue starting at line "
                      + stopIndex);
              code.pr(
                  "{.op="
                      + inst.getOpcode()
                      + ", "
                      + ".rs1="
                      + "EPILOGUE"
                      + ", "
                      + ".rs2="
                      + "-1"
                      + "}"
                      + ",");
              break;
            }
          case BLT:
            {
              InstructionBLT instBLT = (InstructionBLT) inst;
              String rs1Str = "(uint64_t)&" + getVarName(instBLT.rs1, i);
              String rs2Str = "(uint64_t)&" + getVarName(instBLT.rs2, i);
              Phase phase = instBLT.phase;
              String labelString = getWorkerLabelString(phase, i);
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + "Branch to "
                      + labelString
                      + " if "
                      + rs1Str
                      + " < "
                      + rs2Str);
              code.pr(
                  "{.op="
                      + inst.getOpcode()
                      + ", "
                      + ".rs1="
                      + rs1Str
                      + ", "
                      + ".rs2="
                      + rs2Str
                      + ", "
                      + ".rs3="
                      + labelString
                      + "}"
                      + ",");
              break;
            }
          case BNE:
            {
              InstructionBNE instBNE = (InstructionBNE) inst;
              String rs1Str = "(uint64_t)&" + getVarName(instBNE.rs1, i);
              String rs2Str = "(uint64_t)&" + getVarName(instBNE.rs2, i);
              Phase phase = instBNE.phase;
              String labelString = getWorkerLabelString(phase, i);
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + "Branch to "
                      + labelString
                      + " if "
                      + rs1Str
                      + " != "
                      + rs2Str);
              code.pr(
                  "{.op="
                      + inst.getOpcode()
                      + ", "
                      + ".rs1="
                      + rs1Str
                      + ", "
                      + ".rs2="
                      + rs2Str
                      + ", "
                      + ".rs3="
                      + labelString
                      + "}"
                      + ",");
              break;
            }
          case DU:
            {
              TimeValue releaseTime = ((InstructionDU) inst).releaseTime;
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + "Delay Until the variable offset plus "
                      + releaseTime
                      + " is reached.");
              code.pr(
                  "{.op="
                      + inst.getOpcode()
                      + ", "
                      + ".rs1="
                      + "(uint64_t)&"
                      + getVarName(GlobalVarType.WORKER_OFFSET, i)
                      + ", "
                      + ".rs2="
                      + releaseTime.toNanoSeconds()
                      + "LL"
                      + "}"
                      + ",");
              break;
            }
          case EIT:
            {
              ReactionInstance reaction = ((InstructionEIT) inst).reaction;
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + "Execute reaction "
                      + reaction
                      + " if it is marked as queued by the runtime");
              code.pr(
                  "{.op="
                      + inst.getOpcode()
                      + ", "
                      + ".rs1="
                      + reactions.indexOf(reaction)
                      + ", "
                      + ".rs2="
                      + -1
                      + "}"
                      + ",");
              break;
            }
          case EXE:
            {
              ReactionInstance _reaction = ((InstructionEXE) inst).reaction;
              code.pr("// Line " + j + ": " + "Execute reaction " + _reaction);
              code.pr(
                  "{.op="
                      + inst.getOpcode()
                      + ", "
                      + ".rs1="
                      + reactions.indexOf(_reaction)
                      + ", "
                      + ".rs2="
                      + -1
                      + "}"
                      + ",");
              break;
            }
          case JMP:
            {
              Phase target = ((InstructionJMP) inst).target;
              String targetLabel = getWorkerLabelString(target, i);
              code.pr("// Line " + j + ": " + "Jump to label " + targetLabel);
              code.pr(
                  "{.op="
                      + inst.getOpcode()
                      + ", "
                      + ".rs1="
                      + targetLabel
                      + ", "
                      + ".rs2="
                      + 0
                      + "}"
                      + ",");
              break;
            }
          case SAC:
            {
              TimeValue _nextTime = ((InstructionSAC) inst).nextTime;
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + "Sync all workers at this instruction and clear all counters");
              code.pr(
                  "{.op="
                      + inst.getOpcode()
                      + ", "
                      + ".rs1="
                      + "(uint64_t)&"
                      + getVarName(GlobalVarType.WORKER_OFFSET, i)
                      + ", "
                      + ".rs2="
                      + _nextTime.toNanoSeconds()
                      + "LL"
                      + "}"
                      + ",");
              break;
            }
          case STP:
            {
              code.pr("// Line " + j + ": " + "Stop the execution");
              code.pr(
                  "{.op="
                      + inst.getOpcode()
                      + ", "
                      + ".rs1="
                      + -1
                      + ", "
                      + ".rs2="
                      + -1
                      + "}"
                      + ",");
              break;
            }
          case WU:
            {
              int worker = ((InstructionWU) inst).worker;
              int releaseValue = ((InstructionWU) inst).releaseValue;
              code.pr(
                  "// Line "
                      + j
                      + ": "
                      + "Wait until counter "
                      + worker
                      + " reaches "
                      + releaseValue);
              code.pr(
                  "{.op="
                      + inst.getOpcode()
                      + ", "
                      + ".rs1="
                      + worker
                      + ", "
                      + ".rs2="
                      + releaseValue
                      + "}"
                      + ",");
              break;
            }
          default:
            throw new RuntimeException("UNREACHABLE: " + inst.getOpcode());
        }
      }

      code.unindent();
      code.pr("};");
    }

    // Generate an array to store the schedule pointers.
    code.pr("const inst_t* static_schedules[] = {");
    code.indent();
    for (int i = 0; i < instructions.size(); i++) {
      code.pr("schedule_" + i + ",");
    }
    code.unindent();
    code.pr("};");

    // Print to file.
    try {
      code.writeToFile(file.toString());
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  /** Return a C variable name based on the variable type */
  private String getVarName(GlobalVarType type, int worker) {
    switch (type) {
      case GLOBAL_TIMEOUT:
        return "timeout";
      case WORKER_COUNTER:
        return "counters" + "[" + worker + "]";
      case WORKER_OFFSET:
        return "time_offsets" + "[" + worker + "]";
      case EXTERN_START_TIME:
        return "start_time";
      default:
        throw new RuntimeException("UNREACHABLE!");
    }
  }

  /** Return a string of a label for a worker */
  private String getWorkerLabelString(PretVmLabel label, int worker) {
    return "WORKER" + "_" + worker + "_" + label.toString();
  }

  /** Return a string of a label for a worker */
  private String getWorkerLabelString(Phase phase, int worker) {
    return "WORKER" + "_" + worker + "_" + phase.toString();
  }

  /** Pretty printing instructions */
  public void display(PretVmObjectFile objectFile) {
    List<List<Instruction>> instructions = objectFile.getContent();
    for (int i = 0; i < instructions.size(); i++) {
      List<Instruction> schedule = instructions.get(i);
      System.out.println("Worker " + i + ":");
      for (int j = 0; j < schedule.size(); j++) {
        System.out.println(schedule.get(j));
      }
    }
  }

  /**
   * Link multiple object files into a single executable (represented also in an object file class).
   * Instructions are also inserted based on transition guards between fragments. In addition,
   * PREAMBLE and EPILOGUE instructions are inserted here.
   */
  public PretVmExecutable link(List<PretVmObjectFile> pretvmObjectFiles) {

    // Create empty schedules.
    List<List<Instruction>> schedules = new ArrayList<>();
    for (int i = 0; i < workers; i++) {
      schedules.add(new ArrayList<Instruction>());
    }

    // Generate the PREAMBLE code.
    // FIXME: Factor into a separate method.
    for (int i = 0; i < workers; i++) {
      // Configure offset register to be start_time.
      schedules
          .get(i)
          .add(
              new InstructionADDI(
                  GlobalVarType.WORKER_OFFSET, GlobalVarType.EXTERN_START_TIME, 0L));
      // [ONLY WORKER 0]Configure timeout register to be start_time + timeout.
      if (i == 0 && targetConfig.timeout != null) {
        schedules
            .get(i)
            .add(
                new InstructionADDI(
                    GlobalVarType.GLOBAL_TIMEOUT,
                    GlobalVarType.EXTERN_START_TIME,
                    targetConfig.timeout.toNanoSeconds()));
      }
      // Synchronize all workers after finishing PREAMBLE.
      schedules.get(i).add(new InstructionSAC(TimeValue.ZERO));
      // Give the first PREAMBLE instruction to a PREAMBLE label.
      schedules.get(i).get(0).createLabel(Phase.PREAMBLE.toString());
    }

    // Create a queue for storing unlinked object files.
    Queue<PretVmObjectFile> queue = new LinkedList<>();

    // Create a set for tracking state space fragments seen,
    // so that we don't process the same object file twice.
    Set<PretVmObjectFile> seen = new HashSet<>();

    // Start with the first object file, which must not have upstream fragments.
    PretVmObjectFile current = pretvmObjectFiles.get(0);

    // Add the current fragment to the queue.
    queue.add(current);

    // Iterate while there are still object files in the queue.
    while (queue.size() > 0) {

      // Dequeue an object file.
      current = queue.poll();

      // Get the downstream fragments.
      Set<StateSpaceFragment> downstreamFragments = current.getFragment().getDownstreams().keySet();

      // Obtain partial schedules from the current object file.
      List<List<Instruction>> partialSchedules = current.getContent();

      // Append guards for downstream transitions to the partial schedules.
      List<Instruction> defaultTransition = null;
      for (var dsFragment : downstreamFragments) {
        List<Instruction> transition = current.getFragment().getDownstreams().get(dsFragment);
        // Check if a transition is a default transition.
        if (StateSpaceUtils.isDefaultTransition(transition)) {
          defaultTransition = transition;
          continue;
        }
        for (int i = 0; i < workers; i++) {
          partialSchedules.get(i).addAll(transition);
        }
      }
      // Make sure to have the default transition to be appended LAST.
      if (defaultTransition != null) {
        for (int i = 0; i < workers; i++) {
          partialSchedules.get(i).addAll(defaultTransition);
        }
      }

      // Add a label to the first instruction using the exploration phase
      // (INIT, PERIODIC, SHUTDOWN_TIMEOUT, etc.).
      for (int i = 0; i < workers; i++) {
        partialSchedules.get(i).get(0).createLabel(current.getFragment().getPhase().toString());
      }

      // Add the partial schedules to the main schedule.
      for (int i = 0; i < workers; i++) {
        schedules.get(i).addAll(partialSchedules.get(i));
      }

      // Add current to the seen set.
      seen.add(current);

      // Get the object files associated with the downstream fragments.
      Set<PretVmObjectFile> downstreamObjectFiles =
          downstreamFragments.stream()
              .map(StateSpaceFragment::getObjectFile)
              // Filter out null object file since EPILOGUE has a null object file.
              .filter(it -> it != null)
              .collect(Collectors.toSet());

      // Remove object files that have been seen.
      downstreamObjectFiles.removeAll(seen);

      // Add object files related to the downstream fragments to the queue.
      queue.addAll(downstreamObjectFiles);
    }

    // Generate the EPILOGUE code.
    // FIXME: Factor into a separate method.
    for (int i = 0; i < workers; i++) {
      Instruction stp = new InstructionSTP();
      stp.createLabel(Phase.EPILOGUE.toString());
      schedules.get(i).add(stp);
    }

    return new PretVmExecutable(schedules);
  }
}
