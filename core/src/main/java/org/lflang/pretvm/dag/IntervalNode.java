package org.lflang.pretvm.dag;

import org.lflang.TimeValue;

/**
 * Subclass defining a Time Node, which represents a _logical_ time at which downstream reaction
 * nodes are released.
 *
 * @author Shaokai J. Lin
 */
public class IntervalNode extends DagNode {

  //////////////////////////////////////////////////////////////////////
  /// Private Variables

  /** The logical time at which dependent reaction invocations are released */
  private TimeValue interval;

  //////////////////////////////////////////////////////////////////////
  /// Constructor

  /**
   * Constructor
   *
   * @param time the logical release time represented by the node
   */
  public IntervalNode(TimeValue interval) {
    this.interval = interval;
  }

  //////////////////////////////////////////////////////////////////////
  /// Public Methods

  /** Get the logical release time. */
  public TimeValue getInterval() {
    return this.interval;
  }

  /** Set the logical release time. */
  public void setInterval(TimeValue interval) {
    this.interval = interval;
  }

  /** A IntervalNode is synonymous with another if they have the same time. */
  @Override
  public boolean isSynonyous(DagNode that) {
    if (that instanceof IntervalNode node && this.interval.compareTo(node.interval) == 0)
      return true;
    return false;
  }

  @Override
  public String toString() {
    return this.getClass().getSimpleName()
        + " node"
        + (this.interval == null ? "" : " with interval " + this.interval);
  }
}
