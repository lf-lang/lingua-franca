package org.lflang.analyses.scheduler;

import org.lflang.analyses.dag.Dag;

public interface StaticScheduler {
  public Dag partitionDag(Dag dag, int workers, String dotFilePostfix);

  public int setNumberOfWorkers();
}
