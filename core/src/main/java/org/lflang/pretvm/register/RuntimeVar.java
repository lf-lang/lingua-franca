package org.lflang.pretvm.register;

/**
 * Indicates that the register is a variable in the runtime (reactor struct, priority queue, etc.).
 */
public class RuntimeVar extends GlobalRegister {

  /**
   * A C pointer string for which this runtime variable register is created. E.g., This can be a
   * string holding a C pointer pointing to a reactor struct.
   */
  public final String pointer;

  public RuntimeVar(String pointer) {
    super();
    this.pointer = pointer;
  }

  /**
   * RuntimeVar registers can be compared by checking their class names and their pointer strings.
   */
  @Override
  public boolean equals(Object o) {
    if (this == o || (o instanceof RuntimeVar reg && this.pointer.equals(reg.pointer))) return true;
    return false;
  }

  @Override
  public String toString() {
    return "Runtime register " + this.pointer;
  }
}
