package org.lflang.analyses.dag;

/**
 * A helper class defining a pair of DAG nodes
 *
 * 
 */
public class DagNodePair {
  public DagNode key;
  public DagNode value;

  public DagNodePair(DagNode key, DagNode value) {
    this.key = key;
    this.value = value;
  }
}
