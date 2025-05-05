package org.lflang.pretvm.register;

/** Worker-specific addresses to return to after exiting the synchronization code block. */
public class ReturnAddr extends WorkerRegister {
  /** Abstract worker registers whose owner needs to be defined later. */
  public static final ReturnAddr ABSTRACT_REGISTER = new ReturnAddr();

  /** Constructor for an abstract register (no owner assigned) */
  public ReturnAddr() {
    super();
  }

  /** Constructor for an concrete register */
  public ReturnAddr(int worker) {
    super(worker);
  }
}
