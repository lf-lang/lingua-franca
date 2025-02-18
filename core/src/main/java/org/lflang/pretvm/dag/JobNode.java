package org.lflang.pretvm.dag;

import org.lflang.generator.ReactionInstance;

/**
 * Subclass defining a job node, which represents an invocation (job) of a reaction (task). Multiple
 * invocations of the same reaction are represented by multiple nodes.
 *
 * @author Shaokai J. Lin
 */
public class JobNode extends DagNode {

  //////////////////////////////////////////////////////////////////////
  /// Private Variables

  /** If the node type is REACTION, then point the reaction */
  private ReactionInstance reaction;

  /**
   * A DAG node can be associated with a release node, indicating the "timeNode time" of the current
   * node. The release node is one with the maximum tag among all of the upstream release nodes wrt
   * the current node.
   */
  private TimeNode timeNode;

  /**
   * If worker B owns another job node J_B that waits for the current job node, J_A, owned by worker
   * A to finish, worker B needs to block and wait until worker A to reach the progress index before
   * processiong job node J_B. This is necessary to resolve job dependencies in the quasi-static
   * schedule. This progress index is assigned only after the schedule have been determined.
   */
  private Long releaseIndex;

  /**
   * An integer that counts the number of times the same reaction instance has repeated in the
   * graph. The initial value 0 also means this job is not assigned to a worker.
   */
  private int reactionRepeatCount = 0;

  /** Worker ID that owns this job node. The value of -1 means unassigned. */
  private int worker = -1;

  /**
   * A DAG node can be associated with a SYNC node, indicating the "release time" of the current
   * node. The SYNC node is one with the maximum tag among all of the upstream SYNC nodes wrt the
   * current node.
   */
  private TimeNode associatedSyncNode;

  //////////////////////////////////////////////////////////////////////
  /// Constructor

  /**
   * Constructor
   *
   * @param reaction the reaction this node invokes
   */
  public JobNode(ReactionInstance reaction) {
    this.reaction = reaction;
  }

  //////////////////////////////////////////////////////////////////////
  /// Public Methods

  /** A job node is synonymous with another if their reaction instances are the same. */
  public boolean isSynonyous(DagNode that) {
    if (that instanceof JobNode node && this.reaction == node.reaction) return true;
    return false;
  }

  /** Get the progress index. */
  public Long getReleaseIndex() {
    return releaseIndex;
  }

  /** Get the reaction instance this job node invokes. */
  public ReactionInstance getReaction() {
    return this.reaction;
  }

  /** Get the reaction repeat count. */
  public int getReactionRepeatCount() {
    return reactionRepeatCount;
  }

  /** Get the time node that marks the logical release time of this reaction invocation. */
  public DagNode getTimeNode() {
    return timeNode;
  }

  /** Get the worker that owns the job node. */
  public int getWorker() {
    return this.worker;
  }

  /** Set the progress index. */
  public void setReleaseIndex(Long value) {
    releaseIndex = value;
  }

  /** Set the reaction repeat count. */
  public void setReactionRepeatCount(int reactionRepeatCount) {
    this.reactionRepeatCount = reactionRepeatCount;
  }

  /** Set the time node that marks the logical release time of this reaction invocation. */
  public void setTimeNode(TimeNode timeNode) {
    this.timeNode = timeNode;
  }

  /** Set the worker that owns the job node. */
  public void setWorker(int worker) {
    this.worker = worker;
  }

  public TimeNode getAssociatedSyncNode() {
    return associatedSyncNode;
  }

  public void setAssociatedSyncNode(TimeNode syncNode) {
    this.associatedSyncNode = syncNode;
  }

  @Override
  public String toString() {
    return this.getClass().getSimpleName()
        + " node"
        + (this.getReaction() == null ? "" : " for " + this.getReaction())
        + (this.reactionRepeatCount == -1 ? "" : " (repeat: " + this.reactionRepeatCount + ")");
  }
}
