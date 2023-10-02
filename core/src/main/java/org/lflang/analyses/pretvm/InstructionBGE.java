package org.lflang.analyses.pretvm;

import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;

/**
 * Class defining the BGE instruction
 *
 * @author Shaokai Lin
 */
public class InstructionBGE extends InstructionBranchBase {
  public InstructionBGE(GlobalVarType rs1, GlobalVarType rs2, Phase label) {
    super(rs1, rs2, label);
    this.opcode = Opcode.BGE;
  }
  @Override
  public Instruction clone() {
    return new InstructionBGE(rs1, rs2, phase);
  }
}
