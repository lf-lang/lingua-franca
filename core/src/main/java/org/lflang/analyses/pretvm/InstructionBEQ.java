package org.lflang.analyses.pretvm;

import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;

/**
 * Class defining the BEQ instruction
 *
 * @author Shaokai Lin
 */
public class InstructionBEQ extends InstructionBranchBase {
  public InstructionBEQ(Object rs1, Object rs2, Phase label) {
    super(rs1, rs2, label);
    this.opcode = Opcode.BEQ;
  }
}
