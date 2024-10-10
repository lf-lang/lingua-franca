package org.lflang.analyses.pretvm;

import java.util.Objects;

/**
 * Class defining the DU instruction. An worker delays until baseTime + offset.
 *
 * @author Shaokai Lin
 */
public class InstructionDU extends Instruction<Register, Long, Object> {

  public InstructionDU(Register baseTime, Long offset) {
    this.opcode = Opcode.DU;
    this.operand1 = baseTime;
    this.operand2 = offset;
  }

  @Override
  public String toString() {
    return "DU: Delay until Register " + this.operand1 + "'s value + " + this.operand2;
  }

  @Override
  public Instruction<Register, Long, Object> clone() {
    return new InstructionDU(this.operand1, this.operand2);
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionDU that) {
      if (Objects.equals(this.operand1, that.operand1)
          && Objects.equals(this.operand2, that.operand2)) {
        return true;
      } else {
        System.out.println("operand1s equal: " + Objects.equals(this.operand1, that.operand1));
        System.out.println("operand2s equal: " + Objects.equals(this.operand2, that.operand2));
      }
    }
    return false;
  }
}
