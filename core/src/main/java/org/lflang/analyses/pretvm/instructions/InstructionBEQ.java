package org.lflang.analyses.pretvm.instructions;

import org.lflang.analyses.pretvm.Register;

/**
 * Class defining the BEQ instruction
 *
 * 
 */
public class InstructionBEQ extends InstructionBranchBase {
  public InstructionBEQ(Register rs1, Register rs2, Object label) {
    super(rs1, rs2, label);
    this.opcode = Opcode.BEQ;
  }

  @Override
  public Instruction<Register, Register, Object> clone() {
    return new InstructionBEQ(this.operand1, this.operand2, this.operand3);
  }

  @Override
  public String toString() {
    return "Branch to " + this.operand1 + " if " + this.operand2 + " = " + this.operand3;
  }
}
