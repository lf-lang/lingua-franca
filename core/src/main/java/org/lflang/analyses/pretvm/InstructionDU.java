package org.lflang.analyses.pretvm;

import org.lflang.TimeValue;

/**
 * Class defining the DU instruction
 *
 * @author Shaokai Lin
 */
public class InstructionDU extends Instruction<Register,TimeValue,Object> {

  public InstructionDU(Register register, TimeValue releaseTime) {
    this.opcode = Opcode.DU;
    this.operand1 = register;
    this.operand2 = releaseTime; // The physical time point to delay until
  }

  @Override
  public String toString() {
    return "DU: " + this.operand1;
  }

  @Override
  public Instruction<Register,TimeValue,Object> clone() {
    return new InstructionDU(this.operand1, this.operand2);
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionDU that) {
      if (this.opcode == that.opcode
      && this.operand1 == that.operand1
      && this.operand2 == that.operand2
      && this.operand3 == that.operand3) {
        return true;
      }
    }
    return false;
  }
}
