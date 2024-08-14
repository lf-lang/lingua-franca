package org.lflang.analyses.pretvm;

/**
 * Class defining the BLT instruction
 *
 * @author Shaokai Lin
 */
public class InstructionBLT extends InstructionBranchBase {
  public InstructionBLT(Register rs1, Register rs2, Object label) {
    super(rs1, rs2, label);
    this.opcode = Opcode.BLT;
  }

  @Override
  public Instruction<Register,Register,Object> clone() {
    return new InstructionBLT(this.operand1, this.operand2, this.operand3);
  }
}
