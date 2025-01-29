package org.lflang.pretvm.instruction;

import org.lflang.pretvm.register.Register;

/**
 * Class defining the EXE instruction
 *
 * <p>EXE op1 : Execute a function pointer (op1) with an argument pointer (op2). If the function
 * pointer is a reaction, op3 represents the reaction number. Otherwise, op3 is 0.
 *
 * @author Shaokai J. Lin
 */
public class EXE extends Instruction<Register, Register, Integer> {

  /**
   * Constructor
   *
   * @param functionPointer C function pointer to be executed
   * @param functionArgumentPointer A pointer to an argument struct
   * @param reactionNumber A positive reaction priority number if this EXE executes a reaction. 0 if
   *     the EXE executes a helper function.
   */
  public EXE(Register functionPointer, Register functionArgumentPointer, Integer reactionNumber) {
    this.operand1 = functionPointer;
    this.operand2 = functionArgumentPointer;
    this.operand3 = reactionNumber;
  }

  @Override
  public Instruction<Register, Register, Integer> clone() {
    return new EXE(this.operand1, this.operand2, this.operand3);
  }

  @Override
  public String toString() {
    return "EXE : " + this.operand1 + " " + this.operand2 + " " + this.operand3;
  }
}
