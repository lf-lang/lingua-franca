package org.lflang.analyses.dag;

import org.lflang.TimeValue;
import org.lflang.generator.ReactionInstance;

/**
 * Class defining a Dag node.
 *
 * <p>FIXME: Create a base class on top of which dummy, sync, and reaction nodes are defined.
 */
public class DagNode {
  /** Different node types of the DAG */
  public enum dagNodeType {
    DUMMY,
    SYNC,
    REACTION
  }

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

  @Override
  public String toString() {
    return nodeType + " node"
        + (this.timeStep == null ? "" : " @ " + this.timeStep)
        + (this.getReaction() == null ? "" : " for " + this.getReaction());
  }

}
