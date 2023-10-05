package org.lflang.analyses.pretvm;

import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;

/**
 * Class defining the BLT instruction
 *
 * @author Shaokai Lin
 */
public class InstructionBLT extends InstructionBranchBase {
  public InstructionBLT(GlobalVarType rs1, GlobalVarType rs2, Phase label) {
    super(rs1, rs2, label);
    this.opcode = Opcode.BLT;
  }

  @Override
  public Instruction clone() {
    return new InstructionBLT(rs1, rs2, phase);
  }
}
