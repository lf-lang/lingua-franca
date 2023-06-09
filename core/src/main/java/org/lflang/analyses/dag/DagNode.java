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
    public dagNodeType nodeType;

    /** If the node type is REACTION, then point the reaction */
    public ReactionInstance nodeReaction;

    /** 
     * If the node type is Dummy or SYNC, then store the time step, 
     * respectiveley time 
     */
    public TimeValue timeStep;

    /** Worst-Case Execution Time (WCET) of a reaction */
    private TimeValue wcet;

    /** Color of the node for DOT graph */
    private String hexColor = "#FFFFFF";

    /**
     * Constructor. Useful when it is a SYNC or DUMMY node.
     * 
     * @param type node type
     * @param timeStep if the type is DYMMY or SYNC, then record the value
     */
    public DagNode(dagNodeType type, TimeValue timeStep) {
        this.nodeType = type;
        this.timeStep = timeStep;
    }

    /**
     * Constructor. Useful when it is a REACTION node.
     * 
     * @param type node type
     * @param reactionInstance reference to the reaction
     */
    public DagNode(
        dagNodeType type,
        ReactionInstance reactionInstance
    ) {
        this.nodeType = type;
        this.nodeReaction = reactionInstance;
        this.wcet = TimeValue.MAX_VALUE;
    }

    /**
     * Constructor. Useful when it is a REACTION node
     * and the wcet is known.
     * 
     * @param type node type
     * @param reactionInstance reference to the reaction
     */
    public DagNode(
        dagNodeType type,
        ReactionInstance reactionInstance,
        TimeValue wcet
    ) {
        this.nodeType = type;
        this.nodeReaction = reactionInstance;
        this.wcet = wcet;
    }

    public ReactionInstance getReaction() {
        return this.nodeReaction;
    }

    public TimeValue getWCET() {
        return this.wcet;
    }

    public void setWCET(TimeValue wcet) {
        this.wcet = wcet;
    }

    public String getColor() {
        return this.hexColor;
    }

    public void setColor(String hexColor) {
        this.hexColor = hexColor;
    }
}