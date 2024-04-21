package org.lflang.analyses.pretvm;

/**
 * Class defining the BGE instruction
 *
 * @author Shaokai Lin
 */
public class InstructionBGE extends InstructionBranchBase {
  public InstructionBGE(Register rs1, Register rs2, Object label) {
    super(rs1, rs2, label);
    this.opcode = Opcode.BGE;
  }

  @Override
  public Instruction clone() {
    return new InstructionBGE(rs1, rs2, label);
  }
}
