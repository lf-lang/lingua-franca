package org.lflang.pretvm.dag;

import org.lflang.TimeValue;

/**
 * Subclass defining a Time Node, which represents a _logical_ time at which downstream reaction
 * nodes are released.
 *
 * @author Shaokai J. Lin
 */
public class TimeNode extends DagNode implements Comparable<DagNode> {

  //////////////////////////////////////////////////////////////////////
  /// Private Variables

  /** The logical time at which dependent reaction invocations are released */
  private TimeValue time;

  //////////////////////////////////////////////////////////////////////
  /// Constructor

  /**
   * Constructor
   *
   * @param time the logical release time represented by the node
   */
  public TimeNode(TimeValue time) {
    this.time = time;
  }

  //////////////////////////////////////////////////////////////////////
  /// Public Methods

  /**
   * Compare two dag nodes based on their timestamps.
   *
   * @param other The other dag node to compare against.
   * @return -1 if this node has an earlier timestamp than that node, 1 if that node has an earlier
   *     timestamp than this node, 0 if they have the same timestamp.
   */
  @Override
  public int compareTo(DagNode that) {
    if (that instanceof TimeNode node) {
      return TimeValue.compare(this.time, node.time);
    }
    throw new RuntimeException(
        "Only TimeNode can compare with each other. " + that + " is not TimeNode.");
  }

  /** A TimeNode is synonymous with another if they have the same time. */
  @Override
  public boolean isSynonyous(DagNode that) {
    if (that instanceof TimeNode node && this.time.compareTo(node.time) == 0) return true;
    return false;
  }

  /** Get the logical release time. */
  public TimeValue getTime() {
    return this.time;
  }

  /** Set the logical release time. */
  public void setTime(TimeValue time) {
    this.time = time;
  }

  @Override
  public String toString() {
    return this.getClass().getSimpleName() + " node" + (this.time == null ? "" : " @ " + this.time);
  }
}
