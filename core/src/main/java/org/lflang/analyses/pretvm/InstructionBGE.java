package org.lflang.analyses.pretvm;

import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;

/**
 * Class defining the BGE instruction
 *
 * @author Shaokai Lin
 */
public class InstructionBGE extends InstructionBranchBase {
  public InstructionBGE(Object rs1, Object rs2, Phase label) {
    super(rs1, rs2, label);
    this.opcode = Opcode.BGE;
  }
}
