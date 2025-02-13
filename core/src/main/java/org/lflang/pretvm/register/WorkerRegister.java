package org.lflang.pretvm.register;

import java.util.Objects;

public abstract class WorkerRegister extends Register {

  private Integer owner;

  public WorkerRegister() {
    this.global = false;
  }

  public WorkerRegister(Integer worker) {
    this.global = false;
    this.owner = worker;
  }

  public Integer getOwner() {
    return owner;
  }

  /**
   * A worker register is considered abstract if it does not have an owner. This could happen when
   * generating guarded transitions.
   */
  public boolean isAbstract() {
    return owner == null;
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
