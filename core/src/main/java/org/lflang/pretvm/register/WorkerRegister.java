package org.lflang.pretvm.register;

import java.util.Objects;

public abstract class WorkerRegister extends Register {

  public final int owner;

  public WorkerRegister(int worker) {
    this.global = false;
    this.owner = worker;
  }

  /** Worker registers can be compared by checking their class names and their owners. */
  @Override
  public boolean equals(Object o) {
    if (this == o
        || (o instanceof WorkerRegister reg
            && this.getClass() == o.getClass()
            && this.owner == reg.owner)) return true;
    return false;
  }

  /** Compute hash code from both the class name and the owner. */
  @Override
  public int hashCode() {
    return Objects.hash(getClass(), owner);
  }

  @Override
  public String toString() {
    return "Worker " + owner + "'s " + getClass();
  }
}
