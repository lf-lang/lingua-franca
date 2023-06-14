package org.lflang.analyses.evm;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.stream.Collectors;

import org.eclipse.elk.alg.layered.graph.LNode.NodeType;
import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagEdge;
import org.lflang.analyses.dag.DagNode;
import org.lflang.analyses.dag.DagNode.dagNodeType;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.TimerInstance;

public class InstructionGenerator {

  /** A partitioned Dag */
  Dag dag;

  /** Number of workers */
  int workers;

  /** Instructions for all workers */
  List<List<Instruction>> instructions;

  /** Constructor */
	public InstructionGenerator(Dag dagParitioned, int workers) {
    this.dag = dagParitioned;
    this.workers = workers;

    // Initialize instructions array.
    instructions = new ArrayList<>();
    for (int i = 0; i < this.workers; i++) {
      instructions.add(new ArrayList<Instruction>());
    }
	}

  /**
   * Traverse the DAG from head to tail using Khan's algorithm (topological sort).
   */
  public void generate() {
    // Initialize a queue and a map to hold the indegree of each node.
    Queue<DagNode> queue = new LinkedList<>();
    Map<DagNode, Integer> indegree = new HashMap<>();

    // Debug
    int count = 0;

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

      /* Generate instructions for the current node */
      // Get the upstream nodes.
      List<DagNode> upstream = dag.dagEdgesRev
                                  .getOrDefault(current, new HashMap<>())
                                  .keySet()
                                  .stream()
                                  .toList();
      System.out.println("Upstream: " + upstream);

      // If the reaction is triggered by a timer,
      // generate an EXE instructions.
      // FIXME: Handle a reaction triggered by timers and ports.
      if (current.nodeType == dagNodeType.REACTION) {
        ReactionInstance reaction = current.getReaction();
        if (reaction.triggers.stream()
            .anyMatch(trigger -> trigger instanceof TimerInstance)) {
          instructions.get(current.getWorker()).add(new InstructionEXE(reaction));
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
      throw new RuntimeException("The graph has at least one cycle, thus cannot be topologically sorted.");
    }
  }
    
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
