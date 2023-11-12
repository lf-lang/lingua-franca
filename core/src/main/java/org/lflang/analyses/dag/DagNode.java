package org.lflang.analyses.dag;

import org.lflang.TimeValue;
import org.lflang.generator.ReactionInstance;

/**
 * Class defining a Dag node.
 *
 * @author Chadlia Jerad
 * @author Shaokai Lin
 */
public class DagNode {
  /** Different node types of the DAG */
  public enum dagNodeType {
    DUMMY,
    SYNC,
    REACTION
  }

  /**
   * An integer that counts the number of times the same node has occured in the graph. The value 0
   * means unassigned.
   */
  public int count = 0;

  /** Node type */
  public dagNodeType nodeType;

  /** If the node type is REACTION, then point the reaction */
  public ReactionInstance nodeReaction;

  /** If the node type is Dummy or SYNC, then store the time step, respectiveley time */
  public TimeValue timeStep;

  /**
   * Worker ID that owns this node, if this node is a reaction node. The value -1 means unassigned.
   */
  private int worker = -1;

  /** Color of the node for DOT graph */
  private String hexColor = "#FFFFFF";

  /** 
   * A DAG node can be associated with a SYNC node, indicating the "release
   * time" of the current node. The SYNC node is one with the maximum tag among
   * all of the upstream SYNC nodes wrt the current node.
   */
  private DagNode associatedSyncNode;

  /** A debug message in the generated DOT */
  private String dotDebugMsg = "";

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
  public DagNode(dagNodeType type, ReactionInstance reactionInstance) {
    this.nodeType = type;
    this.nodeReaction = reactionInstance;
  }

  public ReactionInstance getReaction() {
    return this.nodeReaction;
  }

  public String getColor() {
    return this.hexColor;
  }

  public void setColor(String hexColor) {
    this.hexColor = hexColor;
  }

  public int getWorker() {
    return this.worker;
  }

  public void setWorker(int worker) {
    this.worker = worker;
  }

  public String getDotDebugMsg() {
    return this.dotDebugMsg;
  }

  public void setDotDebugMsg(String msg) {
    this.dotDebugMsg = msg;
  }

  public boolean isAuxiliary() {
    return (nodeType == dagNodeType.SYNC || nodeType == dagNodeType.DUMMY);
  }

  public int getCount() {
    return count;
  }

  public void setCount(int count) {
    this.count = count;
  }

  public DagNode getAssociatedSyncNode() {
    return associatedSyncNode;
  }

  public void setAssociatedSyncNode(DagNode syncNode) {
    this.associatedSyncNode = syncNode;
  }

  /**
   * A node is synonymous with another if they have the same nodeType, timeStep, and nodeReaction.
   */
  public boolean isSynonyous(DagNode that) {
    if (this.nodeType == that.nodeType
        && (this.timeStep == that.timeStep
            || (this.timeStep != null
                && that.timeStep != null
                && this.timeStep.compareTo(that.timeStep) == 0))
        && this.nodeReaction == that.nodeReaction) return true;
    return false;
  }

  @Override
  public String toString() {
    return nodeType
        + " node"
        + (this.timeStep == null ? "" : " @ " + this.timeStep)
        + (this.getReaction() == null ? "" : " for " + this.getReaction())
        + (this.count == -1 ? "" : " (count: " + this.count + ")");
  }
}
