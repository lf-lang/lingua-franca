package org.lflang.analyses.pretvm.instructions;

import java.util.Objects;
import org.lflang.analyses.pretvm.Register;

/**
 * Class defining the WU instruction
 *
 * 
 */
public class InstructionWU extends Instruction<Register, Long, Object> {

  public InstructionWU(Register register, Long releaseValue) {
    this.opcode = Opcode.WU;
    this.operand1 = register; // A register which the worker waits on
    this.operand2 =
        releaseValue; // The value of the register at which the worker stops spinning and continues
    // executing the schedule
  }

  @Override
  public String toString() {
    return "WU: Wait for " + this.operand1 + " to reach " + this.operand2;
  }

  @Override
  public Instruction<Register, Long, Object> clone() {
    return new InstructionWU(this.operand1, this.operand2);
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionWU that) {
      if (Objects.equals(this.operand1, that.operand1)
          && Objects.equals(this.operand2, that.operand2)) {
        return true;
      }
    }
    return false;
  }
}