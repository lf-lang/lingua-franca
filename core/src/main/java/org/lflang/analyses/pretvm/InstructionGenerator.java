package org.lflang.analyses.pretvm;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.stream.IntStream;
import org.lflang.FileConfig;
import org.lflang.TargetConfig;
import org.lflang.TimeValue;
import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagEdge;
import org.lflang.analyses.dag.DagNode;
import org.lflang.analyses.dag.DagNode.dagNodeType;
import org.lflang.analyses.pretvm.Instruction.Opcode;
import org.lflang.analyses.pretvm.InstructionADDI.TargetVarType;
import org.lflang.analyses.statespace.StateSpaceFragment;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.TimerInstance;

/**
 * A generator that generates PRET VM programs from DAGs
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

    // Add BIT instructions regardless of timeout
    // is specified in the program because it could be
    // specified on the command line.
    // Currently, only generate BIT for the cyclic fragment
    // because the code generation of BIT requires a
    // corresponding STP, which the acyclic fragment does
    // not have. If the acyclic fragment has a STP, then
    // the execution stops before entering the cyclic phase.
    if (fragment.isCyclic()) {
      for (var schedule : instructions) {
        schedule.add(new InstructionBIT());
      }
    }

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

        // If the reaction is triggered by a timer,
        // generate an EXE instruction.
        // FIXME: Handle a reaction triggered by both timers and ports.
        ReactionInstance reaction = current.getReaction();
        if (reaction.triggers.stream().anyMatch(trigger -> trigger instanceof TimerInstance)) {
          instructions.get(current.getWorker()).add(new InstructionEXE(reaction));
        }
        // Otherwise, generate an EIT instruction.
        else {
          instructions.get(current.getWorker()).add(new InstructionEIT(reaction));
        }

        // Increment the counter of the worker.
        instructions.get(current.getWorker()).add(new InstructionADDI(TargetVarType.COUNTER, 1L));
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
                  new InstructionADDI(TargetVarType.OFFSET, current.timeStep.toNanoSeconds()));
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
    if (indegree.values().stream().anyMatch(deg -> deg != 0)) {
      // The graph has at least one cycle.
      throw new RuntimeException(
          "The graph has at least one cycle, thus cannot be topologically sorted.");
    }

    // Add JMP instructions for jumping back to the beginning.
    if (fragment.isCyclic()) {
      for (var schedule : instructions) {
        schedule.add(new InstructionJMP(schedule.get(0))); // Jump to the first instruction.
      }
    }

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

    // Generate variables.
    code.pr("volatile uint32_t " + getCounterVarName(workers) + " = {0};");
    code.pr("volatile instant_t " + getOffsetVarName(workers) + " = {0};");
    code.pr("const size_t num_counters = " + workers + ";");

    // Generate static schedules.
    for (int i = 0; i < instructions.size(); i++) {
      var schedule = instructions.get(i);
      code.pr("const inst_t schedule_" + i + "[] = {");
      code.indent();

      for (int j = 0; j < schedule.size(); j++) {
        Instruction inst = schedule.get(j);
        switch (inst.getOpcode()) {
          case ADDI:
            InstructionADDI addi = (InstructionADDI) inst;
            String varName;
            if (addi.target == TargetVarType.COUNTER) {
              varName = "(uint64_t)&" + getCounterVarName(i);
            } else if (addi.target == TargetVarType.OFFSET) {
              varName = "(uint64_t)&" + getOffsetVarName(i);
            } else {
              throw new RuntimeException("UNREACHABLE");
            }
            code.pr(
                "// Line "
                    + j
                    + ": "
                    + "(Lock-free) increment "
                    + varName
                    + " by "
                    + addi.immediate);
            code.pr(
                "{.op="
                    + addi.getOpcode()
                    + ", "
                    + ".rs1="
                    + varName
                    + ", "
                    + ".rs2="
                    + varName
                    + ", "
                    + ".rs3="
                    + addi.immediate
                    + "LL"
                    + "}"
                    + ",");
            break;
          case ADV2:
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
                    + getOffsetVarName(i)
                    + ", "
                    + ".rs3="
                    + nextTime.toNanoSeconds()
                    + "LL"
                    + "}"
                    + ",");
            break;
          case BIT:
            int stopIndex =
                IntStream.range(0, schedule.size())
                    .filter(k -> (schedule.get(k).getOpcode() == Opcode.STP))
                    .findFirst()
                    .getAsInt();
            code.pr("// Line " + j + ": " + "Branch, if timeout, to line " + stopIndex);
            code.pr(
                "{.op="
                    + inst.getOpcode()
                    + ", "
                    + ".rs1="
                    + stopIndex
                    + ", "
                    + ".rs2="
                    + "-1"
                    + "}"
                    + ",");
            break;
          case DU:
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
                    + getOffsetVarName(i)
                    + ", "
                    + ".rs2="
                    + releaseTime.toNanoSeconds()
                    + "LL"
                    + "}"
                    + ",");
            break;
          case EIT:
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
          case EXE:
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
          case JMP:
            Instruction target = ((InstructionJMP) inst).target;
            int lineNo = schedule.indexOf(target);
            code.pr(
                "// Line "
                    + j
                    + ": "
                    + "Jump to line "
                    + lineNo
                    + " and increment the iteration counter by 1");
            code.pr(
                "{.op="
                    + inst.getOpcode()
                    + ", "
                    + ".rs1="
                    + lineNo
                    + ", "
                    + ".rs2="
                    + 0
                    + "}"
                    + ",");
            break;
          case SAC:
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
                    + getOffsetVarName(i)
                    + ", "
                    + ".rs2="
                    + _nextTime.toNanoSeconds()
                    + "LL"
                    + "}"
                    + ",");
            break;
          case STP:
            code.pr("// Line " + j + ": " + "Stop the execution");
            code.pr(
                "{.op=" + inst.getOpcode() + ", " + ".rs1=" + -1 + ", " + ".rs2=" + -1 + "}" + ",");
            break;
          case WU:
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
          default:
            throw new RuntimeException("UNREACHABLE!");
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

  private String getCounterVarName(int index) {
    return "counters" + "[" + index + "]";
  }

  private String getOffsetVarName(int index) {
    return "time_offsets" + "[" + index + "]";
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
   * In the future, when physical actions are supported, this method will add conditional jumps
   * based on predicates.
   */
  public PretVmExecutable link(List<PretVmObjectFile> pretvmObjectFiles) {

    // Create empty schedules.
    List<List<Instruction>> schedules = new ArrayList<>();
    for (int i = 0; i < workers; i++) {
      schedules.add(new ArrayList<Instruction>());
    }

    // Populate the schedules.
    for (int j = 0; j < pretvmObjectFiles.size(); j++) {
      PretVmObjectFile obj = pretvmObjectFiles.get(j);

      // Make sure the first object file is the entry point,
      // i.e., having no upstream.
      if (j == 0) assert obj.getFragment().getUpstream() == null;

      // Simply stitch all parts together.
      List<List<Instruction>> partialSchedules = obj.getContent();
      for (int i = 0; i < workers; i++) {
        schedules.get(i).addAll(partialSchedules.get(i));
      }
    }

    // Add STP instructions to the end.
    for (int i = 0; i < workers; i++) {
      schedules.get(i).add(new InstructionSTP());
    }

    return new PretVmExecutable(schedules);
  }
}