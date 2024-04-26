package org.lflang.analyses.pretvm;

import java.util.Objects;

public class Register {
    GlobalVarType type;
    Integer owner;

    // Global registers
    public static final Register START_TIME = new Register(GlobalVarType.EXTERN_START_TIME, null);
    public static final Register OFFSET = new Register(GlobalVarType.GLOBAL_OFFSET, null);
    public static final Register OFFSET_INC = new Register(GlobalVarType.GLOBAL_OFFSET_INC, null);
    public static final Register ONE = new Register(GlobalVarType.GLOBAL_ONE, null);
    public static final Register TIMEOUT = new Register(GlobalVarType.GLOBAL_TIMEOUT, null);
    public static final Register ZERO = new Register(GlobalVarType.GLOBAL_ZERO, null);

    // Abstract worker registers whose owner needs to be defined later.
    public static final Register ABSTRACT_WORKER_RETURN_ADDR = new Register(GlobalVarType.WORKER_RETURN_ADDR, null);

    // Placeholder registers that need to be delayed initialized at runtime.
    public static final Register PLACEHOLDER = new Register(GlobalVarType.PLACEHOLDER, null);

    public Register(GlobalVarType type, Integer owner) {
        this.type = type;
        this.owner = owner;
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
        return (owner != null ? "Worker " + owner + "'s " : "") + type;
    }
}

