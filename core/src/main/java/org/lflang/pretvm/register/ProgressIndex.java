package org.lflang.pretvm.register;

/**
 * Worker-specific progress indices to keep track of the progress of workers for implementing
 * "counting locks".
 */
public class ProgressIndex extends WorkerRegister {
  public ProgressIndex(int worker) {
    super(worker);
  }
}
