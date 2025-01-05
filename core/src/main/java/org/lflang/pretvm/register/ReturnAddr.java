package org.lflang.pretvm.register;

/** Worker-specific addresses to return to after exiting the synchronization code block. */
public class ReturnAddr extends WorkerRegister {
  public ReturnAddr(int worker) {
    super(worker);
  }
}
