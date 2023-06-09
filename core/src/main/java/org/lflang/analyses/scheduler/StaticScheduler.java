package org.lflang.analyses.scheduler;

import org.lflang.analyses.dag.Dag;

public interface StaticScheduler {
    public void removeRedundantEdges();
    public void partitionDag();
	public Dag  getDag();
}
