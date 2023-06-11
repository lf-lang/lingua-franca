package org.lflang.analyses.scheduler;

import java.util.ArrayList;
import java.util.HashMap;
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
        ArrayList<KeyValuePair> redundantEdges = new ArrayList<>();
    
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
                            redundantEdges.add(new KeyValuePair(srcNode, destNode));
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
                for (KeyValuePair p : redundantEdges) {
                    HashMap<DagNode, DagEdge> inner3 = dag.dagEdges.get(p.key);
                    if (inner3 != null) {
                        inner3.remove(p.value);
                    }
                }

                // Now that edges have been removed, mark this graph as changed
                // so that the dot file will be regenerated instead of using a cache.
                dag.changed = true;
            }
        }
    }

    @Override
    public void partitionDag(int workers) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'schedule'");
    }

    public class KeyValuePair {
        DagNode key;
        DagNode value;
        public KeyValuePair(DagNode key, DagNode value) {
            this.key = key;
            this.value = value;
        }
    }
}
