package org.lflang.analyses.scheduler;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.Stack;
import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagEdge;
import org.lflang.analyses.dag.DagNode;
import org.lflang.analyses.dag.DagNodePair;

/**
 * A utility class for static scheduler-related methods
 *
 * @author Shaokai Lin
 */
public class StaticSchedulerUtils {

  public static Dag removeRedundantEdges(Dag dagRaw) {
    // Create a copy of the original dag.
    Dag dag = new Dag(dagRaw);

    // List to hold the redundant edges
    ArrayList<DagNodePair> redundantEdges = new ArrayList<>();

    // Iterate over each edge in the graph
    // Add edges
    for (DagNode srcNode : dag.dagEdges.keySet()) {
      HashMap<DagNode, DagEdge> inner = dag.dagEdges.get(srcNode);
      if (inner != null) {
        for (DagNode destNode : inner.keySet()) {
          // Locate the current edge
          DagEdge edge = dag.dagEdges.get(srcNode).get(destNode);

          // Create a visited set to keep track of visited nodes
          Set<DagNode> visited = new HashSet<>();

          // Create a stack for DFS
          Stack<DagNode> stack = new Stack<>();

          // Start from the source node
          stack.push(srcNode);

          // Perform DFS from the source node
          while (!stack.isEmpty()) {
            DagNode currentNode = stack.pop();

            // If we reached the destination node by another path, mark this edge as redundant
            if (currentNode == destNode) {
              // Only mark an edge as redundant if
              // the edge is not coming from a sync node.
              redundantEdges.add(new DagNodePair(srcNode, destNode));
              break;
            }

            if (!visited.contains(currentNode)) {
              visited.add(currentNode);

              // Visit all the adjacent nodes
              for (DagNode srcNode2 : dag.dagEdges.keySet()) {
                HashMap<DagNode, DagEdge> inner2 = dag.dagEdges.get(srcNode2);
                if (inner2 != null) {
                  for (DagNode destNode2 : inner2.keySet()) {
                    DagEdge adjEdge = dag.dagEdges.get(srcNode2).get(destNode2);
                    if (adjEdge.sourceNode == currentNode && adjEdge != edge) {
                      stack.push(adjEdge.sinkNode);
                    }
                  }
                }
              }
            }
          }
        }

        // Remove all the redundant edges
        for (DagNodePair p : redundantEdges) {
          dag.removeEdge(p.key, p.value);
        }
      }
    }

    return dag;
  }

  public static String generateRandomColor() {
    Random random = new Random();
    int r = random.nextInt(256);
    int g = random.nextInt(256);
    int b = random.nextInt(256);

    return String.format("#%02X%02X%02X", r, g, b);
  }

  public static void assignColorsToPartitions(Dag dag) {
    // Assign colors to each partition
    for (int j = 0; j < dag.partitions.size(); j++) {
      List<DagNode> partition = dag.partitions.get(j);
      String randomColor = StaticSchedulerUtils.generateRandomColor();
      for (int i = 0; i < partition.size(); i++) {
        partition.get(i).setColor(randomColor);
        partition.get(i).setWorker(j);
      }
    }
  }
}
