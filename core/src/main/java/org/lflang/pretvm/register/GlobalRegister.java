package org.lflang.pretvm.register;

import java.util.Objects;

public abstract class GlobalRegister extends Register {

  /** Constructor */
  public GlobalRegister() {
    this.global = true;
  }

  /** Global registers can be compared by checking their class names. */
  @Override
  public boolean equals(Object o) {
    if (this == o || (o instanceof GlobalRegister && this.getClass() == o.getClass())) return true;
    return false;
  }

  /** Compute hash code from the class name. */
  @Override
  public int hashCode() {
    return Objects.hash(getClass());
  }

  @Override
  public String toString() {
    return "Global " + getClass();
  }
}
