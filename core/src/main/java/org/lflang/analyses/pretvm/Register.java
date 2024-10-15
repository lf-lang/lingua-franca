package org.lflang.analyses.pretvm;

import java.util.Objects;

public class Register {

  public final RegisterType type;
  public final Integer owner;
  public final String pointer; // Only used for pointers in C structs

  // Constructor for a PretVM register
  public Register(RegisterType type) {
    this.type = type;
    this.owner = null;
    this.pointer = null;
  }

  // Constructor for a PretVM register
  public Register(RegisterType type, Integer owner) {
    this.type = type;
    this.owner = owner;
    this.pointer = null;
  }

  // Constructor for a PretVM register
  public Register(RegisterType type, Integer owner, String pointer) {
    this.type = type;
    this.owner = owner;
    this.pointer = pointer;
  }

  public static Register createRuntimeRegister(String pointer) {
    Register reg = new Register(RegisterType.RUNTIME_STRUCT, null, pointer);
    return reg;
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    if (o == null || getClass() != o.getClass()) return false;
    Register register = (Register) o;
    return Objects.equals(type, register.type)
        && Objects.equals(owner, register.owner)
        && Objects.equals(pointer, register.pointer);
  }

  @Override
  public int hashCode() {
    return Objects.hash(type, owner);
  }

  @Override
  public String toString() {
    // If type is RUNTIME_STRUCT and toString() is called, then simply
    // return the pointer.
    if (type == RegisterType.RUNTIME_STRUCT) return this.pointer;
    // Otherwise, use pretty printing.
    return (owner != null ? "Worker " + owner + "'s " : "") + type;
  }
}
