package org.lflang.analyses.evm;

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
import org.lflang.analyses.evm.Instruction.Opcode;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.TimerInstance;

public class InstructionGenerator {

  /** A partitioned Dag */
  Dag dag;

  /** File configuration */
  FileConfig fileConfig;

  /** Target configuration */
  TargetConfig targetConfig;

  /** Lists for tracking reactor and reaction instances */
  List<ReactorInstance> reactors;

  List<ReactionInstance> reactions;

  /** Number of workers */
  int workers;

  /** Instructions for all workers */
  List<List<Instruction>> instructions;

  /** Constructor */
  public InstructionGenerator(
      Dag dagParitioned,
      FileConfig fileConfig,
      TargetConfig targetConfig,
      List<ReactorInstance> reactors,
      List<ReactionInstance> reactions) {
    this.dag = dagParitioned;
    this.fileConfig = fileConfig;
    this.targetConfig = targetConfig;
    this.workers = targetConfig.workers;
    this.reactors = reactors;
    this.reactions = reactions;

    // Initialize instructions array.
    instructions = new ArrayList<>();
    for (int i = 0; i < this.workers; i++) {
      instructions.add(new ArrayList<Instruction>());
    }
  }

  /** Traverse the DAG from head to tail using Khan's algorithm (topological sort). */
  public void generateInstructions() {
    // Initialize a queue and a map to hold the indegree of each node.
    Queue<DagNode> queue = new LinkedList<>();
    Map<DagNode, Integer> indegree = new HashMap<>();

    // Initialize a reaction index array to keep track of the latest counting
    // lock value for each worker.
    int[] countLockValues = new int[this.workers];

    // Debug
    int count = 0;

    // If timeout is specified, add BIT instructions.
    if (this.targetConfig.timeout != null) {
      for (var schedule : instructions) {
        schedule.add(new InstructionBIT());
      }
    }

    // Initialize indegree of all nodes to be the size of their respective upstream node set.
    for (DagNode node : dag.dagNodes) {
      indegree.put(node, dag.dagEdgesRev.getOrDefault(node, new HashMap<>()).size());
      // Add the node with zero indegree to the queue.
      if (dag.dagEdgesRev.getOrDefault(node, new HashMap<>()).size() == 0) {
        queue.add(node);
      }
    }

    // The main loop for traversal using an iterative topological sort.
    while (!queue.isEmpty()) {
      // Dequeue a node.
      DagNode current = queue.poll();

      // Debug
      current.setDotDebugMsg("count: " + count++);
      System.out.println("Current: " + current);

      // Get the upstream reaction nodes.
      List<DagNode> upstreamReactionNodes =
          dag.dagEdgesRev.getOrDefault(current, new HashMap<>()).keySet().stream()
              .filter(n -> n.nodeType == dagNodeType.REACTION)
              .toList();
      System.out.println("Upstream reaction nodes: " + upstreamReactionNodes);

      // Get the upstream sync nodes.
      List<DagNode> upstreamSyncNodes =
          dag.dagEdgesRev.getOrDefault(current, new HashMap<>()).keySet().stream()
              .filter(n -> n.nodeType == dagNodeType.SYNC)
              .toList();
      System.out.println("Upstream sync nodes: " + upstreamSyncNodes);

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

        // If the reaction depends on a SYNC node,
        // advance to the logical time of the SYNC node first.
        if (upstreamSyncNodes.size() >= 1) {
          if (upstreamSyncNodes.size() > 1)
            System.out.println("WARNING: More than one upstream SYNC nodes detected.");
          instructions
              .get(current.getWorker())
              .add(
                  new InstructionADV2(
                      current.getReaction().getParent(), upstreamSyncNodes.get(0).timeStep));
        }

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
        instructions.get(current.getWorker()).add(new InstructionINC2());
        countLockValues[current.getWorker()]++;

      } else if (current.nodeType == dagNodeType.SYNC) {
        if (current != dag.head && current != dag.tail) {
          // If a worker has reactions that lead to this SYNC node,
          // insert a DU in the schedule.
          // FIXME: Here we have an implicit assumption "logical time is
          // physical time." We need to find a way to relax this assumption.
          for (var i = 0; i < workers; i++) {
            final int j = i; // Need a final int to use the stream method.
            if (upstreamReactionNodes.stream().anyMatch(n -> n.getWorker() == j)) {
              instructions.get(j).add(new InstructionDU(current.timeStep));
            }
          }
        } else if (current == dag.tail) {
          for (var schedule : instructions) {
            schedule.add(new InstructionSAC());
            schedule.add(new InstructionDU(current.timeStep));
          }
        }
      }

      // Visit each downstream node.
      HashMap<DagNode, DagEdge> innerMap = dag.dagEdges.get(current);
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

    // Add JMP and STP instructions.
    for (var schedule : instructions) {
      schedule.add(new InstructionJMP());
      schedule.add(new InstructionSTP());
    }
  }

  /** Generate C code from the instructions list. */
  public void generateCode() {
    // Instantiate a code builder.
    Path srcgen = fileConfig.getSrcGenPath();
    Path file = srcgen.resolve("schedule.c");
    CodeBuilder code = new CodeBuilder();

    // Generate a block comment.
    code.pr(
        String.join(
            "\n",
            "/**",
            " * An auto-generated schedule file for the FS scheduler.",
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
            "#include \"core/threaded/scheduler_instructions.h\""));

    for (int i = 0; i < instructions.size(); i++) {
      var schedule = instructions.get(i);
      code.pr("const inst_t schedule_" + i + "[] = {");
      code.indent();

      for (int j = 0; j < schedule.size(); j++) {
        Instruction inst = schedule.get(j);
        System.out.println("Opcode is " + inst.getOpcode());
        switch (inst.getOpcode()) {
          case ADV2:
            {
              ReactorInstance reactor = ((InstructionADV2) inst).reactor;
              TimeValue nextTime = ((InstructionADV2) inst).nextTime;
              code.pr(
                  "{.op="
                      + inst.getOpcode()
                      + ", "
                      + ".rs1="
                      + reactors.indexOf(reactor)
                      + ", "
                      + ".rs2="
                      + nextTime.toNanoSeconds()
                      + "LL"
                      + "}"
                      + ","
                      + " // (Lock-free) advance the logical time of "
                      + reactor
                      + " to "
                      + nextTime
                      + " wrt the hyperperiod");
              break;
            }
          case BIT:
            {
              int stopIndex =
                  IntStream.range(0, schedule.size())
                      .filter(k -> (schedule.get(k).getOpcode() == Opcode.STP))
                      .findFirst()
                      .getAsInt();
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
                      + ","
                      + " // Branch, if timeout, to line "
                      + stopIndex);
              break;
            }
          case DU:
            {
              TimeValue releaseTime = ((InstructionDU) inst).releaseTime;
              code.pr(
                  "{.op="
                      + inst.getOpcode()
                      + ", "
                      + ".rs1="
                      + releaseTime.toNanoSeconds()
                      + "LL"
                      + ", "
                      + ".rs2="
                      + -1
                      + "}"
                      + ","
                      + " // Delay until physical time reaches "
                      + releaseTime);
              break;
            }
          case EIT:
            {
              ReactionInstance reaction = ((InstructionEIT) inst).reaction;
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
                      + ","
                      + " // Execute reaction "
                      + reaction
                      + " if it is marked as queued by the runtime");
              break;
            }
          case EXE:
            {
              ReactionInstance reaction = ((InstructionEXE) inst).reaction;
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
                      + ","
                      + " // Execute reaction "
                      + reaction);
              break;
            }
          case INC2:
            {
              code.pr(
                  "{.op="
                      + inst.getOpcode()
                      + ", "
                      + ".rs1="
                      + i
                      + ", "
                      + ".rs2="
                      + 1
                      + "}"
                      + ","
                      + " // (Lock-free) increment counter "
                      + i
                      + " by 1");
              break;
            }
            // FIXME: Generalize jump, instead of just jumping to 0.
          case JMP:
            code.pr(
                "{.op="
                    + inst.getOpcode()
                    + ", "
                    + ".rs1="
                    + 0
                    + ", "
                    + ".rs2="
                    + 0
                    + "}"
                    + ","
                    + " // Jump to line 0 and increment the iteration counter by 1");
            break;
          case SAC:
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
                    + ","
                    + " // Sync all workers at this instruction and clear all counters");
            break;
          case STP:
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
                    + ","
                    + " // Stop the execution");
            break;
          case WU:
            int worker = ((InstructionWU) inst).worker;
            int releaseValue = ((InstructionWU) inst).releaseValue;
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
                    + ","
                    + " // Wait until counter "
                    + worker
                    + " reaches "
                    + releaseValue);
            break;
          default:
            // FIXME: Raise an exception.
            System.out.println("UNREACHABLE!");
        }
      }

      code.unindent();
      code.pr("};");
    }

    // Print to file.
    try {
      code.writeToFile(file.toString());
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  /** A getter for the DAG */
  public Dag getDag() {
    return this.dag;
  }

  /** Pretty printing instructions */
  public void display() {
    for (int i = 0; i < this.instructions.size(); i++) {
      List<Instruction> schedule = this.instructions.get(i);
      System.out.println("Worker " + i + ":");
      for (int j = 0; j < schedule.size(); j++) {
        System.out.println(schedule.get(j));
      }
    }
  }
}
