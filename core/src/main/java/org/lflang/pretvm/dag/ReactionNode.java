package org.lflang.pretvm.dag;

import org.lflang.generator.ReactionInstance;

/**
 * Subclass defining a reaction node, which represents a reaction
 * _invocation_. Multiple invocations of the same reaction are
 * represented by multiple nodes.
 *
 * @author Shaokai J. Lin
 */
public class ReactionNode extends Node {
  
  /** If the node type is REACTION, then point the reaction */
  public ReactionInstance reaction;

  /**
   * A DAG node can be associated with a release node, indicating the "releaseNode time" of the current
   * node. The release node is one with the maximum tag among all of the upstream release nodes wrt the
   * current node.
   */
  private ReleaseNode releaseNode;

  /**
   * If the dag node is a REACTION node and there is another node owned by another worker waiting
   * for the current reaction node to finish, the releaseNode value is the number assigned an WU
   * instruction executed by the other worker. The other worker needs to wait until the counter of
   * this worker, who owns this reaction node, reaches releaseValue. We store this information
   * inside a dag node. This value is assigned only after partitions have been determined.
   */
  private Long releaseValue;

  /**
   * Constructor
   *
   * @param reaction the reaction this node invokes
   */
  public ReactionNode(ReactionInstance reaction) {
    this.reaction = reaction;
  }

  public Node getReleaseNode() {
    return releaseNode;
  }

  /** Get the reaction this node invokes */
  public ReactionInstance getReaction() {
    return this.reaction;
  }

  public Long getReleaseValue() {
    return releaseValue;
  }

  public void setReleaseNode(ReleaseNode releaseNode) {
    this.releaseNode = releaseNode;
  }

  public void setReleaseValue(Long value) {
    releaseValue = value;
  }

  /**
   * A reaction node is synonymous with another if their reaction
   * instances are the same.
   */
  @Override
  public boolean isSynonyous(Node that) {
    if (that instanceof ReactionNode node 
      && this.reaction == node.reaction)
      return true;
    return false;
  }

  @Override
  public String toString() {
    return this.getClass().getSimpleName()
        + " node"
        + (this.getReaction() == null ? "" : " for " + this.getReaction())
        + (this.count == -1 ? "" : " (count: " + this.count + ")");
  }
}
