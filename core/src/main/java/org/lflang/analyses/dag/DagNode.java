package org.lflang.analyses.dag;

import org.lflang.TimeValue;
import org.lflang.generator.ReactionInstance;

/**
 *  Different node types of the DAG
 */
enum dagNodeType {
    DUMMY,
    SYNC,
    REACTION
}

/**
 * Class defining a Dag node.
 */
public class DagNode {
    /** Node type */ 
    dagNodeType nodeType;

    /** If the node type is REACTION, then point the reaction */
    ReactionInstance nodeReaction;

    /** 
     * If the node type is Dummy or SYNC, then store the time step, 
     * respectiveley time 
     */
    TimeValue timeStep;


    /**
     * Contructor. Useful when it is a SYNC or DUMMY node.
     * 
     * @param type node type
     * @param timeStep if the type is DYMMY or SYNC, then record the value
     */
    public DagNode(dagNodeType type, TimeValue timeStep) {
        this.nodeType = type;
        this.timeStep = timeStep;
    }

    /**
     * Contructor. Useful when it is a REACTION node.
     * 
     * @param type node type
     * @param reactionInstance reference to the reaction
     */
    public DagNode(dagNodeType type, ReactionInstance reactionInstance) {
        this.nodeType = type;
        this.nodeReaction = reactionInstance;
    }
}