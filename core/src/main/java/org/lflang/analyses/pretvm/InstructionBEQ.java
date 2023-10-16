package org.lflang.analyses.pretvm;

/**
 * Class defining the BEQ instruction
 *
 * @author Shaokai Lin
 */
public class InstructionBEQ extends InstructionBranchBase {
  public InstructionBEQ(Object rs1, Object rs2, Object label) {
    super(rs1, rs2, label);
    this.opcode = Opcode.BEQ;
  }

  @Override
  public Instruction clone() {
    return new InstructionBEQ(rs1, rs2, label);
  }

  @Override
  public String toString() {
    return "Branch to " + label + " if " + rs1 + " = " + rs2;
  }
}
