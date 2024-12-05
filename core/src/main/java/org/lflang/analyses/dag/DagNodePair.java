package org.lflang.analyses.dag;

/**
 * A helper class defining a pair of DAG nodes
 *
 * @author Shaokai Lin
 */
public class DagNodePair {
  public DagNode key;
  public DagNode value;

  public DagNodePair(DagNode key, DagNode value) {
    this.key = key;
    this.value = value;
  }
}
