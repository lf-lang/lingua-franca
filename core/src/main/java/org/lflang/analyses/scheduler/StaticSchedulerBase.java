package org.lflang.analyses.scheduler;

import org.lflang.analyses.dag.Dag;

abstract class StaticSchedulerBase implements StaticScheduler {

  /** A directed acyclic graph (DAG) */
  Dag dag;

  /**
   * Constructor
   *
   * @param dag A directed acyclic graph (DAG)
   */
  public StaticSchedulerBase(Dag dag) {
    this.dag = dag;
  }

  /** Return the DAG. */
  public Dag getDag() {
    return this.dag;
  }

  /**
   * If the number of workers is unspecified, determine a value for the number of workers. This
   * scheduler base class simply returns 1. An advanced scheduler is free to run advanced algorithms
   * here.
   */
  public int setNumberOfWorkers() {
    return 1;
  }
}
