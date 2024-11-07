package org.lflang.analyses.scheduler;

import org.lflang.MessageReporter;
import org.lflang.analyses.dag.Dag;

/**
 * Interface for static scheduler
 *
 * @author Shaokai Lin
 */
public interface StaticScheduler {
  public Dag partitionDag(Dag dag, MessageReporter reporter, int fragmentId, int workers, String filePostfix);

  public int setNumberOfWorkers();
}
