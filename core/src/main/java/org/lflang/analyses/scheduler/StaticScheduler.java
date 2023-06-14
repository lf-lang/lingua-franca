package org.lflang.analyses.scheduler;

import org.lflang.analyses.dag.Dag;

public interface StaticScheduler {
  public void removeRedundantEdges();
  public void partitionDag(int workers);
  public Dag getDag();
}
