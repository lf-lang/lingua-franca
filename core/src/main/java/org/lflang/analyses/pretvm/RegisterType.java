package org.lflang.analyses.pretvm;

/**
 * Types of global variables used by the PRET VM Variable types prefixed by GLOBAL_ are accessible
 * by all workers. Variable types prefixed by WORKER_ mean that there are arrays of these variables
 * such that each worker gets its dedicated variable. For example, COUNTER means that there is an
 * array of counter variables, one for each worker. A worker cannot modify another worker's counter.
 */
public enum RegisterType {
  BINARY_SEMA(false), // Worker-specific binary semaphores to implement synchronization blocks.
  COUNTER(false), // Worker-specific counters to keep track of the progress of a worker, for
  // implementing a "counting lock."
  OFFSET(true), // The current time offset after iterations of hyperperiods
  OFFSET_INC(
      true), // An amount to increment the offset by (usually the current hyperperiod). This is
  // global because worker 0 applies the increment to all workers' offsets.
  ONE(true), // A variable that is always one (i.e., true)
  PLACEHOLDER(true), // Helps the code generator perform delayed instantiation.
  RETURN_ADDR(
      false), // Worker-specific addresses to return to after exiting the synchronization code
  // block.
  RUNTIME_STRUCT(
      true), // Indicates that the variable/register is a field in a runtime-generated struct
  // (reactor struct, priority queue, etc.).
  START_TIME(true), // An external variable to store the start time of the application in epoch time
  TEMP0(false), // A temporary register for each worker
  TEMP1(false), // A temporary register for each worker
  TIMEOUT(true), // A timeout value for all workers
  ZERO(true); // A variable that is always zero (i.e., false)

  /**
   * Whether this variable is shared by all workers. If this is true, then all workers can access
   * and potentially modify the variable. If this is false, then an array will be generated, with
   * each entry accessible by a specific worker.
   */
  private final boolean global;

  /** Constructor */
  RegisterType(boolean global) {
    this.global = global;
  }

  /** Check if the variable is a global variable. */
  public boolean isGlobal() {
    return global;
  }
}
