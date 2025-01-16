package org.lflang.pretvm.instruction;

import org.lflang.pretvm.Label;
import org.lflang.pretvm.register.Register;

/**
 * Class defining the BEQ instruction
 *
 * <p>BEQ op1, op2, op3 : Take the branch (op3) if op1 is equal to op2.
 *
 * @author Shaokai J. Lin
 */
public class BEQ extends Instruction<Register, Register, Label> {

  /**
   * Constructor
   *
   * @param op1 The first operand
   * @param op2 The second operand
   * @param label The label to jump to
   */
  public BEQ(Register op1, Register op2, Label label) {
    this.operand1 = op1;
    this.operand2 = op2;
    this.operand3 = label;
  }

  @Override
  public Instruction<Register, Register, Label> clone() {
    return new BEQ(this.operand1, this.operand2, this.operand3);
  }

  @Override
  public String toString() {
    return "Branch to " + this.operand1 + " if " + this.operand2 + " = " + this.operand3;
  }
}
