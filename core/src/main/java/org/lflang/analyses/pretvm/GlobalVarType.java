package org.lflang.analyses.pretvm;

/**
 * Types of global variables used by the PRET VM Variable types prefixed by GLOBAL_ are accessible
 * by all workers. Variable types prefixed by WORKER_ mean that there are arrays of these variables
 * such that each worker gets its dedicated variable. For example, WORKER_COUNTER means that there
 * is an array of counter variables, one for each worker. A worker cannot modify another worker's
 * counter.
 */
public enum GlobalVarType {
  GLOBAL_TIMEOUT(true), // A timeout value for all workers.
  GLOBAL_OFFSET(true), // The current time offset after iterations of hyperperiods.
  GLOBAL_OFFSET_INC(
      true), // An amount to increment the offset by (usually the current hyperperiod). This is
  // global because worker 0 applies the increment to all workers' offsets.
  GLOBAL_ZERO(true), // A variable that is always zero.
  WORKER_COUNTER(false), // Worker-specific counters to keep track of the progress of a worker, for
  // implementing a "counting lock."
  WORKER_RETURN_ADDR(
      false), // Worker-specific addresses to return to after exiting the synchronization code
  // block.
  WORKER_BINARY_SEMA(
      false), // Worker-specific binary semaphores to implement synchronization blocks.
  EXTERN_START_TIME(
      true); // An external variable to store the start time of the application in epoch time.

  /**
   * Whether this variable is shared by all workers. If this is true, then all workers can access
   * and potentially modify the variable. If this is false, then an array will be generated, with
   * each entry accessible by a specific worker.
   */
  private final boolean shared;

  /** Constructor */
  GlobalVarType(boolean shared) {
    this.shared = shared;
  }

  /** Check if the variable is a shared variable. */
  public boolean isShared() {
    return shared;
  }
}
