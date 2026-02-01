package org.lflang.pretvm.instruction;

import org.lflang.pretvm.Label;
import org.lflang.pretvm.register.Register;

/**
 * Class defining the JAL instruction
 *
 * <p>JAL op1 op2 op3 : Store the return address to op1 and jump to a label (op2) + an additional
 * offset (op3).
 *
 * @author Shaokai J. Lin
 */
public class JAL extends Instruction<Register, Label, Long> {

  /**
   * Constructor
   *
   * @param retAddr A register to store the return address
   * @param targetLabel A target label to jump to
   */
  public JAL(Register retAddr, Label targetLabel) {
    this.operand1 = retAddr;
    this.operand2 = targetLabel;
  }

  /**
   * Constructor
   *
   * @param retAddr A register to store the return address
   * @param targetLabel A target label to jump to
   * @param offset An additional offset
   */
  public JAL(Register retAddr, Label targetLabel, Long offset) {
    this(retAddr, targetLabel);
    this.operand3 = offset;
  }

  @Override
  public Instruction<Register, Label, Long> clone() {
    return new JAL(this.operand1, this.operand2, this.operand3);
  }

  @Override
  public String toString() {
    return "JAL: "
        + "store return address in "
        + this.operand1
        + " and jump to "
        + this.operand2
        + (this.operand3 == null ? "" : " + " + this.operand3);
  }
}
