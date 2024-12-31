package org.lflang.pretvm.dag;

import org.lflang.TimeValue;

/**
 * Subclass defining a release node, which represents a _logical_ time at which downstream reaction
 * nodes are released.
 *
 * @author Shaokai J. Lin
 */
public class ReleaseNode extends Node implements Comparable<Node> {

  /** The logical time at which dependent reaction invocations are released */
  public TimeValue time;

  /**
   * Constructor
   *
   * @param time the logical release time represented by the node
   */
  public ReleaseNode(TimeValue time) {
    this.time = time;
  }

  /**
   * Compare two dag nodes based on their timestamps.
   *
   * @param other The other dag node to compare against.
   * @return -1 if this node has an earlier timestamp than that node, 1 if that node has an earlier
   *     timestamp than this node, 0 if they have the same timestamp.
   */
  @Override
  public int compareTo(Node that) {
    if (that instanceof ReleaseNode node) {
      return TimeValue.compare(this.time, node.time);
    }
    throw new RuntimeException(
        "Only ReleaseNode can compare with each other. " + that + " is not ReleaseNode.");
  }

  /** A ReleaseNode is synonymous with another if they have the same time. */
  @Override
  public boolean isSynonyous(Node that) {
    if (that instanceof ReleaseNode node && this.time.compareTo(node.time) == 0) return true;
    return false;
  }

  @Override
  public String toString() {
    return this.getClass().getSimpleName()
        + " node"
        + (this.time == null ? "" : " @ " + this.time)
        + (this.count == -1 ? "" : " (count: " + this.count + ")");
  }
}
