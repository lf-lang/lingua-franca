package org.lflang.pretvm.instruction;

import org.lflang.pretvm.register.Register;

/**
 * Class defining the DU instruction.
 *
 * <p>DU op1, op2 : Delay until a physical timepoint (op1) plus an offset (op2) is reached.
 *
 * @author Shaokai J. Lin
 */
public class DU extends Instruction<Register, Long, Object> {

  public DU(Register baseTime, Long offset) {
    this.operand1 = baseTime;
    this.operand2 = offset;
  }

  @Override
  public Instruction<Register, Long, Object> clone() {
    return new DU(this.operand1, this.operand2);
  }

  @Override
  public String toString() {
    return "DU: Delay until Register " + this.operand1 + "'s value + " + this.operand2;
  }
}
