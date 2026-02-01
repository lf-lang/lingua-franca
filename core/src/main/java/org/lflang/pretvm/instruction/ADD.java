package org.lflang.pretvm.instruction;

import org.lflang.pretvm.register.Register;

/**
 * Class defining the ADD instruction.
 *
 * <p>ADD op1, op2, op3 : Add to an integer variable (op2) by an integer variable (op3) and store
 * the result in a destination variable (op1).
 *
 * @author Shaokai J. Lin
 */
public class ADD extends Instruction<Register, Register, Register> {

  public ADD(Register target, Register source, Register source2) {
    this.operand1 = target;
    this.operand2 = source;
    this.operand3 = source2;
  }

  @Override
  public Instruction<Register, Register, Register> clone() {
    return new ADD(this.operand1, this.operand2, this.operand3);
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
}
