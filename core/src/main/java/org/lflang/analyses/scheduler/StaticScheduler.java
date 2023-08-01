package org.lflang.analyses.scheduler;

import org.lflang.analyses.dag.Dag;

/**
 * Interface for static scheduler
 *
 * @author Shaokai Lin
 */
public interface StaticScheduler {
  public Dag partitionDag(Dag dag, int workers, String dotFilePostfix);

  public int setNumberOfWorkers();
}
