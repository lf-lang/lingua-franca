package org.lflang.pretvm.instruction;

import org.lflang.pretvm.register.Register;

/**
 * Class defining the ADDI instruction
 *
 * <p>ADDI op1, op2, op3 : Add to an integer variable (op2) by an immediate (op3) and store the
 * result in a destination variable (op1).
 *
 * @author Shaokai J. Lin
 */
public class ADDI extends Instruction<Register, Register, Long> {

  public ADDI(Register target, Register source, Long immediate) {
    this.operand1 = target; // The target register
    this.operand2 = source; // The source register
    this.operand3 = immediate; // The immediate to be added with the variable
  }

  @Override
  public Instruction<Register, Register, Long> clone() {
    return new ADDI(this.operand1, this.operand2, this.operand3);
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
