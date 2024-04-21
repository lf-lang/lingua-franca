package org.lflang.analyses.pretvm;

/**
 * Class defining the BNE instruction
 *
 * @author Shaokai Lin
 */
public class InstructionBNE extends InstructionBranchBase {
  public InstructionBNE(Register rs1, Register rs2, Object label) {
    super(rs1, rs2, label);
    this.opcode = Opcode.BNE;
  }

  @Override
  public Instruction clone() {
    return new InstructionBNE(rs1, rs2, label);
  }
}
