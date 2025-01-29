package org.lflang.pretvm.instruction;

import org.lflang.pretvm.register.Register;

/**
 * Class defining the JALR instruction
 *
 * <p>JALR op1, op2, op3 : Store the return address in destination (op1) and jump to baseAddr (op2)
 * + offset (op3)
 *
 * @author Shaokai J. Lin
 */
public class JALR extends Instruction<Register, Register, Long> {

  /**
   * Constructor
   *
   * @param destination A destination register to return to
   * @param baseAddr A register containing the base address
   * @param offset A immediate representing the address offset
   */
  public JALR(Register destination, Register baseAddr, Long offset) {
    this.operand1 = destination;
    this.operand2 = baseAddr;
    this.operand3 = offset;
  }

  @Override
  public Instruction<Register, Register, Long> clone() {
    return new JALR(this.operand1, this.operand2, this.operand3);
  }

  @Override
  public String toString() {
    return "JALR: "
        + "store return address in "
        + this.operand1
        + " and jump to "
        + this.operand2
        + (this.operand3 == null ? "" : " + " + this.operand3);
  }
}
