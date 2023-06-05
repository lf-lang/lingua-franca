
package org.lflang.analyses.statespace;

import java.util.ArrayList;

import org.lflang.TimeValue;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;


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


    /**
     * Constructor. Sets the amin reactor and initializes the dag
     * @param main main reactor instance
     */    
    public DagGenerator(ReactorInstance main) {
        this.main = main;
        this.dag = new Dag();
    }

    /**
     * Generates the Dag.
     * It starts by calling StateSpaceExplorer to construct the state space
     * diagram. This latter, together with the lf program topology and priorities
     * are used to generate the Dag.
     */
    public void DagGenerate(){
        // Start first by exploring the state space to constrcut the diagram
        StateSpaceExplorer stateSpaceExplorer =  new StateSpaceExplorer(this.main);

        // FIXME: What value for horizon? 
        stateSpaceExplorer.explore(new Tag(0, 0, true), true);

        // Then get the diagram
        this.stateSpaceDiagram = stateSpaceExplorer.getStateSpaceDiagram();

        // Parse the state space diagram
        StateSpaceNode currentStateSpaceNode = this.stateSpaceDiagram.head;
        TimeValue previousTime = TimeValue.ZERO;
        DagNode previousSync = null; 
        while (currentStateSpaceNode != null) {
            TimeValue time = currentStateSpaceNode.time;
            // Add SYNC node 
            DagNode sync = this.dag.AddNode(dagNodeType.SYNC, time);

            // Create DUMMY and Connect SYNC and previous SYNC to DUMMY
            if (! previousTime.equals(TimeValue.ZERO)) {
                TimeValue timeDiff = time.sub(previousTime);
                DagNode dummy = this.dag.AddNode(dagNodeType.DUMMY, timeDiff);
                this.dag.AddEdge(previousSync, dummy);
                this.dag.AddEdge(dummy, sync);
            }

            // Add ReactionsInvoqued nodes, as well as the edges connecting
            // them to SYNC
            for (ReactionInstance ri : currentStateSpaceNode.reactionsInvoked) {
                DagNode node = this.dag.AddNode(dagNodeType.REACTION, ri);
                this.dag.AddEdge(sync, node);
            }

            // Now add the reactions dependencies
            for (ReactionInstance ri : currentStateSpaceNode.reactionsInvoked) {
                // WIP
            }

            // The stop condition is when the tail node is encountred
            if (currentStateSpaceNode == this.stateSpaceDiagram.tail) {
                // FIXME: Add the last DYMMY and SYNC nodes

                break;
            } 

            // Move to the next state space            
            currentStateSpaceNode = stateSpaceDiagram.getDownstreamNode(currentStateSpaceNode);
            previousSync = sync;
            previousTime = time;
        } 
        
        // Now iterate over the lf diagram to report the dependencies
        
    }

    /**
     * Parses the Dag and constructs the dependency matrix
     */
    public void DependencyMatrixGenerator () {
        if (this.dag.isEmpty()) 
            DagGenerate();

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
}