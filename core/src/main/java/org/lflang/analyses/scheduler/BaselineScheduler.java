package org.lflang.analyses.scheduler;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import java.util.Stack;

import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagEdge;
import org.lflang.analyses.dag.DagNode;

public class BaselineScheduler extends StaticSchedulerBase {

    public BaselineScheduler(Dag dagRaw) {
		super(dagRaw);
	}

    @Override
    public void removeRedundantEdges() {
        // List to hold the redundant edges
        ArrayList<DagEdge> redundantEdges = new ArrayList<>();
    
        // Iterate over each edge in the graph
        for (DagEdge edge : dag.dagEdges) {
            DagNode srcNode = edge.sourceNode;
            DagNode destNode = edge.sinkNode;
    
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
                    redundantEdges.add(edge);
                    break;
                }
    
                if (!visited.contains(currentNode)) {
                    visited.add(currentNode);
    
                    // Visit all the adjacent nodes
                    for (DagEdge adjEdge : dag.dagEdges) {
                        if (adjEdge.sourceNode == currentNode && adjEdge != edge) {
                            stack.push(adjEdge.sinkNode);
                        }
                    }
                }
            }
        }
    
        // Remove all the redundant edges
        System.out.println(redundantEdges);
        dag.dagEdges.removeAll(redundantEdges);

        // Now that edges have been removed, mark this graph as changed
        // so that the dot file will be regenerated instead of using a cache.
        dag.changed = true;
    }
    
    @Override
    public void partitionDag() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'schedule'");
    }
    
}
