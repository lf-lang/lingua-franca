package org.lflang.pretvm.instruction;

import org.lflang.pretvm.register.Register;

/**
 * Class defining the WU instruction
 *
 * <p>WU op1, op2 : Wait until a variable (op1) to be greater than or equal to a desired value
 * (op2).
 *
 * @author Shaokai J. Lin
 */
public class WU extends Instruction<Register, Long, Object> {

  /**
   * Constructor
   *
   * @param register A register which the worker waits on
   * @param releaseValue The value of the register at which the worker stops spinning and continues
   *     executing the schedule
   */
  public WU(Register register, Long releaseValue) {
    this.operand1 = register;
    this.operand2 = releaseValue;
  }

  @Override
  public Instruction<Register, Long, Object> clone() {
    return new WU(this.operand1, this.operand2);
  }

  @Override
  public String toString() {
    return "WU: Wait for " + this.operand1 + " to be greater than or equal to " + this.operand2;
  }
}
