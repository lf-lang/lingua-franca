package org.lflang.analyses.scheduler;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.Stack;
import java.util.stream.Collectors;

import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagEdge;
import org.lflang.analyses.dag.DagNode;
import org.lflang.analyses.dag.DagNode.dagNodeType;
import org.lflang.generator.c.CFileConfig;

public class BaselineScheduler extends StaticSchedulerBase {

    /** File config */
    protected final CFileConfig fileConfig;

    public BaselineScheduler(Dag dag, CFileConfig fileConfig) {
		super(dag);
        this.fileConfig = fileConfig;
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
            }
        }
    }

    public static String generateRandomColor() {
        Random random = new Random();
        int r = random.nextInt(256);
        int g = random.nextInt(256);
        int b = random.nextInt(256);

        return String.format("#%02X%02X%02X", r, g, b);
    }

    public class Worker {
        private long totalWCET = 0;
        private List<DagNode> tasks = new ArrayList<>();
    
        public void addTask(DagNode task) {
            tasks.add(task);
            totalWCET += task.getReaction().wcet.toNanoSeconds();
        }
    
        public long getTotalWCET() {
            return totalWCET;
        }
    }
    
    public void partitionDag(int numWorkers) {

        // Prune redundant edges.
        removeRedundantEdges();
        Dag dag = getDag();
        
        // Generate a dot file.
        Path srcgen = fileConfig.getSrcGenPath();
        Path file = srcgen.resolve("dag_pruned.dot");
        dag.generateDotFile(file);

        // Initialize workers
        Worker[] workers = new Worker[numWorkers];
        for (int i = 0; i < numWorkers; i++) {
            workers[i] = new Worker();
        }
    
        // Sort tasks in descending order by WCET
        List<DagNode> reactionNodes = dag.dagNodes.stream()
                        .filter(node -> node.nodeType == dagNodeType.REACTION)
                        .collect(Collectors.toCollection(ArrayList::new));
        reactionNodes.sort(Comparator.comparing((DagNode node) -> node.getReaction().wcet.toNanoSeconds()).reversed());

    
        // Assign tasks to workers
        for (DagNode node : reactionNodes) {
            // Find worker with least work
            Worker minWorker = Arrays.stream(workers).min(Comparator.comparing(Worker::getTotalWCET)).orElseThrow();
    
            // Assign task to this worker
            minWorker.addTask(node);
        }

        // Update partitions
        for (int i = 0; i < numWorkers; i++) {
            dag.partitions.add(workers[i].tasks);
        }

        // Assign colors to each partition
        for (int j = 0; j < dag.partitions.size(); j++) {
            List<DagNode> partition = dag.partitions.get(j);
            String randomColor = generateRandomColor();
            for (int i = 0; i < partition.size(); i++) {
                partition.get(i).setColor(randomColor);
                partition.get(i).setWorker(j);
            }
        }

        // Generate another dot file.
        Path file2 = srcgen.resolve("dag_partitioned.dot");
        dag.generateDotFile(file2);
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
