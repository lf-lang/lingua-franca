package org.lflang.analyses.statespace;

/**
 * Class defining a Dag edge.
 */
public class DagEdge {
    /** The source DAG node */
    DagNode sourceNode;

    /** The sink DAG node */
    DagNode sinkNode;
    
    ////////////////////////////////////////
    //// Public constructor

    /**
     * Contructor of a DAG edge
     * 
     * @param source the source DAG node 
     * @param sink the sink DAG node 
     */
    public DagEdge(DagNode source, DagNode sink) {
        this.sourceNode = source;
        this.sinkNode = sink;
    }
}