package org.lflang.analyses.scheduler;

import org.lflang.analyses.dag.Dag;

abstract class StaticSchedulerBase implements StaticScheduler {

  Dag dag;

  // FIXME: store the number of workers.

  public StaticSchedulerBase(Dag dag) {
    this.dag = dag;
  }

  public Dag getDag() {
    return this.dag;
  }
}
