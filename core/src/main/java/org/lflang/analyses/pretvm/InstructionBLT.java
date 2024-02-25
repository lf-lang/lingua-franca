package org.lflang.analyses.pretvm;

/**
 * Class defining the BLT instruction
 *
 * @author Shaokai Lin
 */
public class InstructionBLT extends InstructionBranchBase {
  public InstructionBLT(Object rs1, Object rs2, Object label) {
    super(rs1, rs2, label);
    this.opcode = Opcode.BLT;
  }

  @Override
  public Instruction clone() {
    return new InstructionBLT(rs1, rs2, label);
  }
}
