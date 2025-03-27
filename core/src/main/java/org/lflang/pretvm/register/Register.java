package org.lflang.pretvm.register;

/**
 * Types of global variables used by the PretVM Variable types prefixed by GLOBAL_ are accessible by
 * all workers. Variable types prefixed by WORKER_ mean that there are arrays of these variables
 * such that each worker gets its dedicated variable. For example, RETURN_ADDR means that there is
 * an array of return address variables, one for each worker. A worker cannot modify another
 * worker's return address.
 *
 * @author Shaokai J. Lin
 */
public abstract class Register {

  /**
   * Whether this variable is shared by all workers. If this is true, then all workers can access
   * and potentially modify the variable. If this is false, then an array will be generated, with
   * each entry accessible by a specific worker.
   */
  protected boolean global;

  /** Check if the variable is a global variable. */
  public boolean isGlobal() {
    return global;
  }
}
