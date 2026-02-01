package org.lflang.pretvm.dag;

/** A pair of DAG nodes. */
public class DagNodePair {
  public DagNode key;
  public DagNode value;

  public DagNodePair(DagNode key, DagNode value) {
    this.key = key;
    this.value = value;
  }
}
