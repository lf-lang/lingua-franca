package org.lflang.analyses.pretvm;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.stream.Collectors;
import org.lflang.FileConfig;
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
import org.lflang.target.TargetConfig;
import org.lflang.target.property.FastProperty;
import org.lflang.target.property.TimeOutProperty;

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
    Long[] countLockValues = new Long[workers];
    Arrays.fill(countLockValues, 0L); // Initialize all elements to 0

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

      /* Generate instructions for the current node */
      if (current.nodeType == dagNodeType.REACTION) {

        // Get the nearest upstream sync node.
        DagNode associatedSyncNode = current.getAssociatedSyncNode();

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
                .add(
                    new InstructionWU(
                        GlobalVarType.WORKER_COUNTER,
                        upstreamOwner,
                        countLockValues[upstreamOwner]));
          }
        }

        // If the reaction depends on a single SYNC node,
        // advance to the LOGICAL time of the SYNC node first,
        // as well as delay until the PHYSICAL time indicated by the SYNC node.
        // Skip if it is the head node since this is done in SAC.
        // FIXME: Here we have an implicit assumption "logical time is
        // physical time." We need to find a way to relax this assumption.
        if (associatedSyncNode != null && associatedSyncNode != dagParitioned.head) {
          // Generate an ADVI instruction.
          instructions
              .get(current.getWorker())
              .add(
                  new InstructionADVI(
                      current.getReaction().getParent(),
                      GlobalVarType.GLOBAL_OFFSET,
                      associatedSyncNode.timeStep.toNanoSeconds()));
          // Generate a DU instruction if fast mode is off.
          if (!targetConfig.get(FastProperty.INSTANCE)) {
            instructions
                .get(current.getWorker())
                .add(new InstructionDU(associatedSyncNode.timeStep));
          }
        }

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
                    GlobalVarType.WORKER_COUNTER,
                    current.getWorker(),
                    GlobalVarType.WORKER_COUNTER,
                    current.getWorker(),
                    1L));
        countLockValues[current.getWorker()]++;

      } else if (current.nodeType == dagNodeType.SYNC) {
        if (current == dagParitioned.tail) {
          // When the timeStep = TimeValue.MAX_VALUE in a SYNC node,
          // this means that the DAG is acyclic and can end without
          // real-time constraints, hence we do not genereate SAC,
          // DU, and ADDI.
          if (current.timeStep != TimeValue.MAX_VALUE) {
            for (int worker = 0; worker < workers; worker++) {
              List<Instruction> schedule = instructions.get(worker);
              // Add a DU instruction if fast mode is off.
              if (!targetConfig.get(FastProperty.INSTANCE))
                schedule.add(new InstructionDU(current.timeStep));
              // [Only Worker 0] Update the time increment register.
              if (worker == 0) {
                schedule.add(
                    new InstructionADDI(
                        GlobalVarType.GLOBAL_OFFSET_INC,
                        null,
                        GlobalVarType.GLOBAL_ZERO,
                        null,
                        current.timeStep.toNanoSeconds()));
              }
              // Let all workers go to SYNC_BLOCK after finishing PREAMBLE.
              schedule.add(new InstructionJAL(GlobalVarType.WORKER_RETURN_ADDR, Phase.SYNC_BLOCK));
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

    return new PretVmObjectFile(instructions, fragment);
  }

  // FIXME: Instead of finding this manually, we can store this information when
  // building the DAG.
  private DagNode findNearestUpstreamSync(
      DagNode node, Map<DagNode, HashMap<DagNode, DagEdge>> dagEdgesRev) {
    if (node.nodeType == dagNodeType.SYNC) {
      return node;
    }

    HashMap<DagNode, DagEdge> upstreamNodes = dagEdgesRev.getOrDefault(node, new HashMap<>());
    for (DagNode upstreamNode : upstreamNodes.keySet()) {
      DagNode result = findNearestUpstreamSync(upstreamNode, dagEdgesRev);
      if (result != null) {
        return result;
      }
    }

    return null;
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
    if (targetConfig.isSet(TimeOutProperty.INSTANCE))
      // FIXME: Why is timeout volatile?
      code.pr(
          "volatile uint64_t "
              + getVarName(GlobalVarType.GLOBAL_TIMEOUT, null)
              + " = "
              + targetConfig.get(TimeOutProperty.INSTANCE).toNanoSeconds()
              + "LL"
              + ";");
    code.pr("const size_t num_counters = " + workers + ";"); // FIXME: Seems unnecessary.
    code.pr("volatile reg_t " + getVarName(GlobalVarType.GLOBAL_OFFSET, workers) + " = 0;");
    code.pr("volatile reg_t " + getVarName(GlobalVarType.GLOBAL_OFFSET_INC, null) + " = 0;");
    code.pr("const uint64_t " + getVarName(GlobalVarType.GLOBAL_ZERO, null) + " = 0;");
    code.pr(
        "volatile uint64_t "
            + getVarName(GlobalVarType.WORKER_COUNTER, workers)
            + " = {0};"); // Must be uint64_t, otherwise writing a long long to it could cause
    // buffer overflow.
    code.pr("volatile reg_t " + getVarName(GlobalVarType.WORKER_RETURN_ADDR, workers) + " = {0};");
    code.pr("volatile reg_t " + getVarName(GlobalVarType.WORKER_BINARY_SEMA, workers) + " = {0};");

    // Generate static schedules. Iterate over the workers (i.e., the size
    // of the instruction list).
    for (int worker = 0; worker < instructions.size(); worker++) {
      var schedule = instructions.get(worker);
      code.pr("const inst_t schedule_" + worker + "[] = {");
      code.indent();

      for (int j = 0; j < schedule.size(); j++) {
        Instruction inst = schedule.get(j);

        // If there is a label attached to the instruction, generate a comment.
        if (inst.hasLabel()) code.pr("// " + getWorkerLabelString(inst.getLabel(), worker) + ":");

        // Generate code based on opcode
        switch (inst.getOpcode()) {
          case ADD:
            {
              InstructionADD add = (InstructionADD) inst;
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{.opcode="
                      + add.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + "&"
                      + getVarName(add.target, add.targetOwner)
                      + ", "
                      + ".op2.reg="
                      + "(reg_t*)"
                      + "&"
                      + getVarName(add.source, add.sourceOwner)
                      + ", "
                      + ".op3.reg="
                      + "(reg_t*)"
                      + "&"
                      + getVarName(add.source2, add.source2Owner)
                      + "}"
                      + ",");
              break;
            }
          case ADDI:
            {
              InstructionADDI addi = (InstructionADDI) inst;
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{.opcode="
                      + addi.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + "&"
                      + getVarName(addi.target, addi.targetOwner)
                      + ", "
                      + ".op2.reg="
                      + "(reg_t*)"
                      + "&"
                      + getVarName(addi.source, addi.sourceOwner)
                      + ", "
                      + ".op3.imm="
                      + addi.immediate
                      + "LL"
                      + "}"
                      + ",");
              break;
            }
          case ADV:
            {
              ReactorInstance reactor = ((InstructionADV) inst).reactor;
              GlobalVarType baseTime = ((InstructionADV) inst).baseTime;
              GlobalVarType increment = ((InstructionADV) inst).increment;
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{.opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.imm="
                      + reactors.indexOf(reactor)
                      + ", "
                      + ".op2.reg="
                      + "(reg_t*)"
                      + "&"
                      + getVarName(baseTime, worker)
                      + ", "
                      + ".op3.reg="
                      + "(reg_t*)"
                      + "&"
                      + getVarName(increment, worker)
                      + "}"
                      + ",");
              break;
            }
          case ADVI:
            {
              ReactorInstance reactor = ((InstructionADVI) inst).reactor;
              GlobalVarType baseTime = ((InstructionADVI) inst).baseTime;
              Long increment = ((InstructionADVI) inst).increment;
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{.opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.imm="
                      + reactors.indexOf(reactor)
                      + ", "
                      + ".op2.reg="
                      + "(reg_t*)"
                      + "&"
                      + getVarName(baseTime, worker)
                      + ", "
                      + ".op3.imm="
                      + increment
                      + "LL" // FIXME: Why longlong should be ULL for our type?
                      + "}"
                      + ",");
              break;
            }
          case BEQ:
            {
              InstructionBEQ instBEQ = (InstructionBEQ) inst;
              String rs2Str = "&" + getVarName(instBEQ.rs2, worker);
              String rs1Str = "&" + getVarName(instBEQ.rs1, worker);
              Phase phase = instBEQ.phase;
              String labelString = getWorkerLabelString(phase, worker);
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
                  "{.opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + rs1Str
                      + ", "
                      + ".op2.reg="
                      + rs2Str
                      + ", "
                      + ".op3.imm="
                      + labelString
                      + "}"
                      + ",");
              break;
            }
          case BGE:
            {
              InstructionBGE instBGE = (InstructionBGE) inst;
              String rs1Str = "&" + getVarName(instBGE.rs1, worker);
              String rs2Str = "&" + getVarName(instBGE.rs2, worker);
              Phase phase = instBGE.phase;
              String labelString = getWorkerLabelString(phase, worker);
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
                  "{.opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + rs1Str
                      + ", "
                      + ".op2.reg="
                      + rs2Str
                      + ", "
                      + ".op3.imm="
                      + labelString
                      + "}"
                      + ",");
              break;
            }
          case BLT:
            {
              InstructionBLT instBLT = (InstructionBLT) inst;
              String rs1Str = "&" + getVarName(instBLT.rs1, worker);
              String rs2Str = "&" + getVarName(instBLT.rs2, worker);
              Phase phase = instBLT.phase;
              String labelString = getWorkerLabelString(phase, worker);
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
                  "{.opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + rs1Str
                      + ", "
                      + ".op2.reg="
                      + rs2Str
                      + ", "
                      + ".op3.imm="
                      + labelString
                      + "}"
                      + ",");
              break;
            }
          case BNE:
            {
              InstructionBNE instBNE = (InstructionBNE) inst;
              String rs1Str = "&" + getVarName(instBNE.rs1, worker);
              String rs2Str = "&" + getVarName(instBNE.rs2, worker);
              Phase phase = instBNE.phase;
              String labelString = getWorkerLabelString(phase, worker);
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
                  "{.opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + rs1Str
                      + ", "
                      + ".op2.reg="
                      + rs2Str
                      + ", "
                      + ".op3.imm="
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
                  "{.opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + "&"
                      + getVarName(GlobalVarType.GLOBAL_OFFSET, null)
                      + ", "
                      + ".op2.imm="
                      + releaseTime.toNanoSeconds()
                      + "LL" // FIXME: LL vs ULL. Since we are giving time in signed ints. Why not
                      // use signed int as our basic data type not, unsigned?
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
                  "{.opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.imm="
                      + reactions.indexOf(reaction)
                      + ", "
                      + ".op2.imm="
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
                  "{.opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.imm="
                      + reactions.indexOf(_reaction)
                      + ", "
                      + ".op2.imm="
                      + -1
                      + "}"
                      + ",");
              break;
            }
          case JAL:
            {
              GlobalVarType retAddr = ((InstructionJAL) inst).retAddr;
              Phase target = ((InstructionJAL) inst).target;
              String targetLabel = getWorkerLabelString(target, worker);
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{.opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + "&"
                      + getVarName(retAddr, worker)
                      + ", "
                      + ".op2.imm="
                      + targetLabel
                      + "}"
                      + ",");
              break;
            }
          case JALR:
            {
              GlobalVarType destination = ((InstructionJALR) inst).destination;
              GlobalVarType baseAddr = ((InstructionJALR) inst).baseAddr;
              Long immediate = ((InstructionJALR) inst).immediate;
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{.opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + "&"
                      + getVarName(destination, worker)
                      + ", "
                      + ".op2.reg=" // FIXME: This does not seem right op2 seems to be used as an
                      // immediate...
                      + "(reg_t*)"
                      + "&"
                      + getVarName(baseAddr, worker)
                      + ", "
                      + ".op3.imm="
                      + immediate
                      + "}"
                      + ",");
              break;
            }
          case STP:
            {
              code.pr("// Line " + j + ": " + "Stop the execution");
              code.pr("{.opcode=" + inst.getOpcode() + "}" + ",");
              break;
            }
          case WLT:
            {
              GlobalVarType variable = ((InstructionWLT) inst).variable;
              int owner = ((InstructionWLT) inst).owner;
              Long releaseValue = ((InstructionWLT) inst).releaseValue;
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{.opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + "&"
                      + getVarName(variable, owner)
                      + ", "
                      + ".op2.imm="
                      + releaseValue
                      + "}"
                      + ",");
              break;
            }
          case WU:
            {
              GlobalVarType variable = ((InstructionWU) inst).variable;
              int owner = ((InstructionWU) inst).owner;
              Long releaseValue = ((InstructionWU) inst).releaseValue;
              code.pr("// Line " + j + ": " + inst.toString());
              code.pr(
                  "{.opcode="
                      + inst.getOpcode()
                      + ", "
                      + ".op1.reg="
                      + "(reg_t*)"
                      + "&"
                      + getVarName(variable, owner)
                      + ", "
                      + ".op2.imm="
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
  private String getVarName(GlobalVarType type, Integer worker) {
    switch (type) {
      case GLOBAL_TIMEOUT:
        return "timeout";
      case GLOBAL_OFFSET:
        return "time_offset";
      case GLOBAL_OFFSET_INC:
        return "offset_inc";
      case GLOBAL_ZERO:
        return "zero";
      case WORKER_COUNTER:
        return "counters" + "[" + worker + "]";
      case WORKER_RETURN_ADDR:
        return "return_addr" + "[" + worker + "]";
      case WORKER_BINARY_SEMA:
        return "binary_sema" + "[" + worker + "]";
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

    // Generate and append the PREAMBLE code.
    List<List<Instruction>> preamble = generatePreamble();
    for (int i = 0; i < schedules.size(); i++) {
      schedules.get(i).addAll(preamble.get(i));
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
        // Add COPIES of guarded transitions to the partial schedules.
        // They have to be copies since otherwise labels created for different
        // workers will be added to the same instruction object, creating conflicts.
        for (int i = 0; i < workers; i++) {
          partialSchedules.get(i).addAll(transition.stream().map(Instruction::clone).toList());
        }
      }
      // Make sure to have the default transition copies to be appended LAST.
      if (defaultTransition != null) {
        for (int i = 0; i < workers; i++) {
          partialSchedules
              .get(i)
              .addAll(defaultTransition.stream().map(Instruction::clone).toList());
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
    List<List<Instruction>> epilogue = generateEpilogue();
    for (int i = 0; i < schedules.size(); i++) {
      schedules.get(i).addAll(epilogue.get(i));
    }

    // Generate and append the synchronization block.
    List<List<Instruction>> syncBlock = generateSyncBlock();
    for (int i = 0; i < schedules.size(); i++) {
      schedules.get(i).addAll(syncBlock.get(i));
    }

    return new PretVmExecutable(schedules);
  }

  /** Generate the PREAMBLE code. */
  private List<List<Instruction>> generatePreamble() {

    List<List<Instruction>> schedules = new ArrayList<>();
    for (int worker = 0; worker < workers; worker++) {
      schedules.add(new ArrayList<Instruction>());
    }

    for (int worker = 0; worker < workers; worker++) {
      // [ONLY WORKER 0] Configure timeout register to be start_time + timeout.
      if (worker == 0) {
        // Configure offset register to be start_time.
        schedules
            .get(worker)
            .add(
                new InstructionADDI(
                    GlobalVarType.GLOBAL_OFFSET, null, GlobalVarType.EXTERN_START_TIME, null, 0L));
        // Configure timeout if needed.
        if (targetConfig.get(TimeOutProperty.INSTANCE) != null) {
          schedules
              .get(worker)
              .add(
                  new InstructionADDI(
                      GlobalVarType.GLOBAL_TIMEOUT,
                      worker,
                      GlobalVarType.EXTERN_START_TIME,
                      worker,
                      targetConfig.get(TimeOutProperty.INSTANCE).toNanoSeconds()));
        }
        // Update the time increment register.
        schedules
            .get(worker)
            .add(
                new InstructionADDI(
                    GlobalVarType.GLOBAL_OFFSET_INC, null, GlobalVarType.GLOBAL_ZERO, null, 0L));
      }
      // Let all workers go to SYNC_BLOCK after finishing PREAMBLE.
      schedules
          .get(worker)
          .add(new InstructionJAL(GlobalVarType.WORKER_RETURN_ADDR, Phase.SYNC_BLOCK));
      // Give the first PREAMBLE instruction to a PREAMBLE label.
      schedules.get(worker).get(0).createLabel(Phase.PREAMBLE.toString());
    }

    return schedules;
  }

  /** Generate the EPILOGUE code. */
  private List<List<Instruction>> generateEpilogue() {

    List<List<Instruction>> schedules = new ArrayList<>();
    for (int worker = 0; worker < workers; worker++) {
      schedules.add(new ArrayList<Instruction>());
    }

    for (int worker = 0; worker < workers; worker++) {
      Instruction stp = new InstructionSTP();
      stp.createLabel(Phase.EPILOGUE.toString());
      schedules.get(worker).add(stp);
    }

    return schedules;
  }

  /** Generate the synchronization code block. */
  private List<List<Instruction>> generateSyncBlock() {

    List<List<Instruction>> schedules = new ArrayList<>();

    for (int w = 0; w < workers; w++) {

      schedules.add(new ArrayList<Instruction>());

      // Worker 0 will be responsible for changing the global variables while
      // the other workers wait.
      if (w == 0) {

        // Wait for non-zero workers' binary semaphores to be set to 1.
        for (int worker = 1; worker < workers; worker++) {
          schedules.get(w).add(new InstructionWU(GlobalVarType.WORKER_BINARY_SEMA, worker, 1L));
        }

        // Update the global time offset by an increment (typically the hyperperiod).
        schedules
            .get(0)
            .add(
                new InstructionADD(
                    GlobalVarType.GLOBAL_OFFSET,
                    null,
                    GlobalVarType.GLOBAL_OFFSET,
                    null,
                    GlobalVarType.GLOBAL_OFFSET_INC,
                    null));

        // Reset all workers' counters.
        for (int worker = 0; worker < workers; worker++) {
          schedules
              .get(w)
              .add(
                  new InstructionADDI(
                      GlobalVarType.WORKER_COUNTER, worker, GlobalVarType.GLOBAL_ZERO, null, 0L));
        }

        // Advance all reactors' tags to offset + increment.
        for (int j = 0; j < this.reactors.size(); j++) {
          schedules
              .get(w)
              .add(new InstructionADVI(this.reactors.get(j), GlobalVarType.GLOBAL_OFFSET, 0L));
        }

        // Set non-zero workers' binary semaphores to be set to 0.
        for (int worker = 1; worker < workers; worker++) {
          schedules
              .get(w)
              .add(
                  new InstructionADDI(
                      GlobalVarType.WORKER_BINARY_SEMA,
                      worker,
                      GlobalVarType.GLOBAL_ZERO,
                      null,
                      0L));
        }

        // Jump back to the return address.
        schedules
            .get(0)
            .add(
                new InstructionJALR(
                    GlobalVarType.GLOBAL_ZERO, GlobalVarType.WORKER_RETURN_ADDR, 0L));

      } else {

        // Set its own semaphore to be 1.
        schedules
            .get(w)
            .add(
                new InstructionADDI(
                    GlobalVarType.WORKER_BINARY_SEMA, w, GlobalVarType.GLOBAL_ZERO, null, 1L));

        // Wait for the worker's own semaphore to be less than 1.
        schedules.get(w).add(new InstructionWLT(GlobalVarType.WORKER_BINARY_SEMA, w, 1L));

        // Jump back to the return address.
        schedules
            .get(w)
            .add(
                new InstructionJALR(
                    GlobalVarType.GLOBAL_ZERO, GlobalVarType.WORKER_RETURN_ADDR, 0L));
      }

      // Give the first instruction to a SYNC_BLOCK label.
      schedules.get(w).get(0).createLabel(Phase.SYNC_BLOCK.toString());
    }

    return schedules;
  }
}
