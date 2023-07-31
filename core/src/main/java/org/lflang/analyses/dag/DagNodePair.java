package org.lflang.analyses.dag;

public class DagNodePair {
    public DagNode key;
    public DagNode value;

    public DagNodePair(DagNode key, DagNode value) {
        this.key = key;
        this.value = value;
    }
}
