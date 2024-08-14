package org.lflang.analyses.pretvm;

import java.util.Objects;

public class Register {

    // PretVM global registers
    public static final Register START_TIME = new Register(GlobalVarType.EXTERN_START_TIME, null);
    public static final Register OFFSET = new Register(GlobalVarType.GLOBAL_OFFSET, null);
    public static final Register OFFSET_INC = new Register(GlobalVarType.GLOBAL_OFFSET_INC, null);
    public static final Register ONE = new Register(GlobalVarType.GLOBAL_ONE, null);
    public static final Register TIMEOUT = new Register(GlobalVarType.GLOBAL_TIMEOUT, null);
    public static final Register ZERO = new Register(GlobalVarType.GLOBAL_ZERO, null);

    // Abstract worker registers whose owner needs to be defined later.
    public static final Register ABSTRACT_WORKER_RETURN_ADDR = new Register(GlobalVarType.WORKER_RETURN_ADDR, null);

    public final GlobalVarType type;
    public final Integer owner;
    public final String pointer; // Only used for pointers in C structs

    // Constructor for a PretVM register
    public Register(GlobalVarType type, Integer owner) {
        this.type = type;
        this.owner = owner;
        this.pointer = "";
    }

    // Use this constructor if we know the concrete address of a field in a
    // reactor struct.
    // FIXME: The usage of this is a little confusing, because this is also used
    // for auxiliary function pointers, which is not necessarily in the
    // generated runtime struct but directly written in schedule.c.
    public Register(String pointer) {
        this.type = GlobalVarType.RUNTIME_STRUCT;
        this.owner = null;
        this.pointer = pointer;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Register register = (Register) o;
        return Objects.equals(type, register.type) && Objects.equals(owner, register.owner);
    }

    @Override
    public int hashCode() {
        return Objects.hash(type, owner);
    }

    @Override
    public String toString() {
        // If type is RUNTIME_STRUCT and toString() is called, then simply
        // return the pointer.
        if (type == GlobalVarType.RUNTIME_STRUCT) return this.pointer;
        // Otherwise, use pretty printing.
        return (owner != null ? "Worker " + owner + "'s " : "") + type;
    }
}

