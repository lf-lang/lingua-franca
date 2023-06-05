
package org.lflang.analyses.dag;

import java.util.ArrayList;

import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.analyses.statespace.StateSpaceNode;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CFileConfig;

/**
 * Constructs a Directed Acyclic Graph (Dag) from the State Space Diagram.
 * This is part of the static schedule generation.
 *
 * @author Chadlia Jerad
 * @author Shaokai Lin
 */
public class DagGenerator {
    /** The main reactor instance. */
    public ReactorInstance main;

    /** The Dag to be contructed. */
    public Dag dag;

    /**
     * State Space Diagram, to be constructed by explorer() method in 
     * StateSpaceExplorer.
     */
    public StateSpaceDiagram stateSpaceDiagram;

    /** Adjacency Matrix */
    public Integer[][] adjacencyMatrix;

    /** Array of node labels */
    public ArrayList<String> nodeLabels;

    /** Array of some of the execution times */
    public long[] maxExecutionTimes;

    /** File config */
    protected final CFileConfig fileConfig;

    /**
     * A dot file that represents the diagram
     */
    private CodeBuilder dot;

    /**
     * Constructor. Sets the amin reactor and initializes the dag
     * @param main main reactor instance
     */    
    public DagGenerator(
        CFileConfig fileConfig,
        ReactorInstance main,
        StateSpaceDiagram stateSpaceDiagram
    ) {
        this.fileConfig = fileConfig;
        this.main = main;
        this.stateSpaceDiagram = stateSpaceDiagram;
        this.dag = new Dag();
    }

    /**
     * Generates the Dag.
     * It starts by calling StateSpaceExplorer to construct the state space
     * diagram. This latter, together with the lf program topology and priorities
     * are used to generate the Dag.
     */
    public void generateDag(){
        // Variables
        StateSpaceNode currentStateSpaceNode = this.stateSpaceDiagram.head;
        TimeValue previousTime = TimeValue.ZERO;
        DagNode previousSync = null;
        int loopNodeReached = 0;
        boolean lastIteration = false;
        ArrayList<DagNode> allReactionNodes = new ArrayList<>();

        while (currentStateSpaceNode != null) {
            // Check if the current node is a loop node.
            // The stop condition is when the loop node is encountered the 2nd time.
            if (currentStateSpaceNode == this.stateSpaceDiagram.loopNode)
                loopNodeReached++;
            if (currentStateSpaceNode == this.stateSpaceDiagram.loopNode
                && loopNodeReached >= 2) {
                // Add previous nodes' edges to the last SYNC node.
                lastIteration = true;
            }

            // Get the current logical time. Or, if this is the last iteration,
            // set the loop period as the logical time.
            TimeValue time;
            if (!lastIteration)
                time = currentStateSpaceNode.time;
            else
                time = new TimeValue(this.stateSpaceDiagram.loopPeriod, TimeUnit.NANO);

            // Add SYNC node 
            DagNode sync = this.dag.addNode(dagNodeType.SYNC, time);

            // Create DUMMY and Connect SYNC and previous SYNC to DUMMY
            if (! time.equals(TimeValue.ZERO)) {
                TimeValue timeDiff = time.sub(previousTime);
                DagNode dummy = this.dag.addNode(dagNodeType.DUMMY, timeDiff);
                this.dag.addEdge(previousSync, dummy);
                this.dag.addEdge(dummy, sync);
            }

            // Do not add more reaction nodes, and add edges 
            // from existing reactions to the last node.
            if (lastIteration) {
                for (DagNode n : allReactionNodes) {
                    this.dag.addEdge(n, sync);
                }
                break;
            }

            // Add reaction nodes, as well as the edges connecting them to SYNC.
            ArrayList<DagNode> currentReactionNodes = new ArrayList<>();
            for (ReactionInstance reaction : currentStateSpaceNode.reactionsInvoked) {
                DagNode node = this.dag.addNode(dagNodeType.REACTION, reaction);
                currentReactionNodes.add(node);
                this.dag.addEdge(sync, node);
            }

            // If there is a newly released reaction found and its prior
            // invocation is not closed, close the previous invocation to
            // preserve a deterministic order.
            // FIXME: Replace with a stream method.
            ArrayList<ReactionInstance> currentReactions = new ArrayList<>();
            for (DagNode n : currentReactionNodes) {
                currentReactions.add(n.nodeReaction);
            }
            for (DagNode n : allReactionNodes) {
                if (currentReactions.contains(n.nodeReaction)) {
                    this.dag.addEdge(n, sync);
                }
            }

            // Then add all the current reaction nodes to the list of all
            // previously seen reaction invocations.
            allReactionNodes.addAll(currentReactionNodes);

            // Now add the reactions dependencies
            for (DagNode n1 : currentReactionNodes) {
                for (DagNode n2 : currentReactionNodes) {
                    if (n1.nodeReaction.dependentReactions().contains(n2.nodeReaction)) {
                        this.dag.addEdge(n1, n2);
                    }
                }
            }

            // Move to the next state space            
            currentStateSpaceNode = stateSpaceDiagram.getDownstreamNode(currentStateSpaceNode);
            previousSync = sync;
            previousTime = time;
        } 

        // Add the concluding node.

        
        // Now iterate over the lf diagram to report the dependencies
        
    }

    /**
     * Parses the Dag and constructs the dependency matrix
     */
    public void generateDependencyMatrix () {
        if (this.dag.isEmpty()) {
            System.out.println("The DAG is empty. No matrix generated.");
            return;
        }

        // Create the adjacency matrix, the node labels array and the maximum
        // execution timwes array 
        int size = this.dag.dagNodes.size();
        adjacencyMatrix = new Integer[size][size];
        nodeLabels = new ArrayList<String>();
        maxExecutionTimes = new long[size];

        // Iterate over the nodes to record their names and their max execution
        // times If there is no exeution time, then 0 will be recorded.
        // Also, initialize the matrix with 0.
        for (int dnIndex = 0 ; dnIndex < size ; dnIndex++) {
            DagNode dn = dag.dagNodes.get(dnIndex);
            if (dn.nodeType == dagNodeType.SYNC) {
                nodeLabels.add("Sync");
                maxExecutionTimes[dnIndex] = 0;
            } else if (dn.nodeType == dagNodeType.DUMMY) {
                nodeLabels.add("Dummy");
                maxExecutionTimes[dnIndex] = dn.timeStep.time;
            } else { // This is a reaction node
                nodeLabels.add(dn.nodeReaction.getFullName());
                maxExecutionTimes[dnIndex] = 0;
            }
            
            // Initialize the matrix to 0
            for (int index = 0 ; index < size ; index++) {
                adjacencyMatrix[dnIndex][index] = 0;
            }
        }

        // Fill the '1' in the adjency matrix, whenever there is and edge
        for (DagEdge de : dag.dagEdges) {
            int indexSource = dag.dagNodes.indexOf(de.sourceNode);
            int indexSink = dag.dagNodes.indexOf(de.sinkNode);
            adjacencyMatrix[indexSource][indexSink] = 1;
        }
        
        // Now, all quantities are ready to be saved in a file
        // ...
    }

    // A getter for the DAG
    public Dag getDag() {
        return this.dag;
    }

    /**
     * Generate a dot file from the state space diagram.
     * 
     * An example dot file:
digraph dag {
    fontname="Calibri"
    rankdir=TB;
    node [shape = circle, width = 1.5, height = 1.5, fixedsize = true];
    ranksep=1.0;  // Increase distance between ranks
    nodesep=1.0;  // Increase distance between nodes in the same rank
    
    0 [label="Sync@0ms", style="dotted"];
    1 [label="Dummy=5ms", style="dotted"];
    2 [label="Sync@5ms", style="dotted"];
    3 [label="Dummy=5ms", style="dotted"];
    4 [label="Sync@10ms", style="dotted"];
    
    // Here we are adding a new subgraph that contains nodes we want aligned.
    {
        rank = same;
        0; 1; 2; 3; 4;
    }
    
    5 [label="sink.0\nWCET=0.1ms\nEST=0.3ms", fillcolor=green, style=filled];
    6 [label="source.0\nWCET=0.3ms\nEST=0ms", fillcolor=green, style=filled];
    7 [label="source2.0\nWCET=0.3ms\nEST=0ms", fillcolor=red, style=filled];
    8 [label="sink.1\nWCET=0.1ms\nEST=0.4ms", fillcolor=green, style=filled];
    9 [label="sink.2\nWCET=0.1ms\nEST=0.5ms", fillcolor=green, style=filled];
    10 [label="sink.0\nWCET=0.1ms\nEST=5ms", fillcolor=green, style=filled];

    0 -> 1;
    1 -> 2;
    2 -> 3;
    3 -> 4;
    
    0 -> 6;
    6 -> 5;
    0 -> 7;
    5 -> 2;
    5 -> 8;
    8 -> 9;
    7 -> 9;
    9 -> 10;
    2 -> 10;
    10 -> 4;
}
     * 
     * @return a CodeBuilder with the generated code
     */
    public CodeBuilder generateDot() {
        if (dot == null) {
            dot = new CodeBuilder();
            dot.pr("digraph DAG {");
            dot.indent();
            
            // Graph settings
            dot.pr("fontname=\"Calibri\";");
            dot.pr("rankdir=TB;");
            dot.pr("node [shape = circle, width = 1.5, height = 1.5, fixedsize = true];");
            dot.pr("ranksep=3.0;  // Increase distance between ranks");
            dot.pr("nodesep=3.0;  // Increase distance between nodes in the same rank");
            
            // Define nodes.
            ArrayList<Integer> auxiliaryNodes = new ArrayList<>();
            for (int i = 0; i < this.dag.dagNodes.size(); i++) {
                DagNode node = this.dag.dagNodes.get(i);
                String code = "";
                String label = "";
                if (node.nodeType == dagNodeType.SYNC) {
                    label = "label=\"Sync" + "@" + node.timeStep + "\", style=\"dotted\"";
                    auxiliaryNodes.add(i);
                } else if (node.nodeType == dagNodeType.DUMMY) {
                    label = "label=\"Dummy" + "=" + node.timeStep + "\", style=\"dotted\"";
                    auxiliaryNodes.add(i);
                } else if (node.nodeType == dagNodeType.REACTION) {
                    label = "label=\"" + node.nodeReaction.getFullName() + "\nWCET=?ms\"";
                } else {
                    // Raise exception.
                    System.out.println("UNREACHABLE");
                    System.exit(1);
                }
                code += i + "[" + label + "]";
                dot.pr(code);
            }

            // Align auxiliary nodes.
            dot.pr("{");
            dot.indent();
            dot.pr("rank = same;");
            for (Integer i : auxiliaryNodes) {
                dot.pr(i + "; ");
            }
            dot.unindent();
            dot.pr("}");

            // Add edges
            for (DagEdge e : this.dag.dagEdges) {
                int sourceIdx = this.dag.dagNodes.indexOf(e.sourceNode);
                int sinkIdx   = this.dag.dagNodes.indexOf(e.sinkNode);
                dot.pr(sourceIdx + " -> " + sinkIdx);
            }

            dot.unindent();
            dot.pr("}");
        }
        return this.dot;
    }
}