package org.lflang.pretvm.instruction;

import java.util.Objects;
import org.lflang.pretvm.Label;
import org.lflang.pretvm.register.Register;

/** Base class for branch instructions (BEQ, BGE, BLT, BNE). Operands are two registers and a label. */
public abstract class BranchBase extends Instruction<Register, Register, Label> {

  public BranchBase(Register rs1, Register rs2, Label label) {
    this.operand1 = rs1;
    this.operand2 = rs2;
    this.operand3 = label;
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof BranchBase that) {
      return Objects.equals(this.operand1, that.operand1)
          && Objects.equals(this.operand2, that.operand2)
          && Objects.equals(this.operand3, that.operand3);
    }
    return false;
  }
}
