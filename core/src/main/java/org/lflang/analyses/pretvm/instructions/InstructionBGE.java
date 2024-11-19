package org.lflang.analyses.pretvm.instructions;

import org.lflang.analyses.pretvm.Register;

/**
 * Class defining the BGE instruction
 *
 * 
 */
public class InstructionBGE extends InstructionBranchBase {
  public InstructionBGE(Register rs1, Register rs2, Object label) {
    super(rs1, rs2, label);
    this.opcode = Opcode.BGE;
  }

  @Override
  public Instruction<Register, Register, Object> clone() {
    return new InstructionBGE(this.operand1, this.operand2, this.operand3);
  }
}
