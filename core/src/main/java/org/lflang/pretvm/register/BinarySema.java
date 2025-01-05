package org.lflang.pretvm.register;

/** Worker-specific binary semaphores to implement synchronization blocks. */
public class BinarySema extends WorkerRegister {
  public BinarySema(int worker) {
    super(worker);
  }
}
