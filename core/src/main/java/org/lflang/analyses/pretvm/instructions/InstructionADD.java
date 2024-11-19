package org.lflang.analyses.pretvm.instructions;

import java.util.Objects;
import org.lflang.analyses.pretvm.Register;

/**
 * Class defining the ADD instruction
 *
 * 
 */
public class InstructionADD extends Instruction<Register, Register, Register> {

  public InstructionADD(Register target, Register source, Register source2) {
    this.opcode = Opcode.ADD;
    this.operand1 = target;
    this.operand2 = source;
    this.operand3 = source2;
  }

  @Override
  public String toString() {
    return "Increment " + this.operand1 + " by adding " + this.operand2 + " and " + this.operand3;
  }

  @Override
  public Instruction<Register, Register, Register> clone() {
    return new InstructionADD(this.operand1, this.operand2, this.operand3);
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionADD that) {
      if (Objects.equals(this.operand1, that.operand1)
          && Objects.equals(this.operand2, that.operand2)
          && Objects.equals(this.operand3, that.operand3)) {
        return true;
      }
    }
    return false;
  }
}