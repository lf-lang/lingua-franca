package org.lflang.analyses.pretvm.instructions;

import java.util.Objects;
import org.lflang.analyses.pretvm.Register;

/**
 * Class defining the ADDI instruction
 *
 * 
 */
public class InstructionADDI extends Instruction<Register, Register, Long> {

  public InstructionADDI(Register target, Register source, Long immediate) {
    this.opcode = Opcode.ADDI;
    this.operand1 = target; // The target register
    this.operand2 = source; // The source register
    this.operand3 = immediate; // The immediate to be added with the variable
  }

  @Override
  public String toString() {
    return "Increment "
        + this.operand1
        + " by adding "
        + this.operand2
        + " and "
        + this.operand3
        + "LL";
  }

  @Override
  public Instruction<Register, Register, Long> clone() {
    return new InstructionADDI(this.operand1, this.operand2, this.operand3);
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionADDI that) {
      if (Objects.equals(this.operand1, that.operand1)
          && Objects.equals(this.operand2, that.operand2)
          && Objects.equals(this.operand3, that.operand3)) {
        return true;
      }
    }
    return false;
  }
}
