package org.lflang.analyses.pretvm;

import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;

/**
 * Class defining the BNE instruction
 *
 * @author Shaokai Lin
 */
public class InstructionBNE extends InstructionBranchBase {
  public InstructionBNE(GlobalVarType rs1, GlobalVarType rs2, Phase label) {
    super(rs1, rs2, label);
    this.opcode = Opcode.BNE;
  }
}
