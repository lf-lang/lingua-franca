package org.lflang.analyses.pretvm;

import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;

/**
 * Class defining the BLT instruction
 *
 * @author Shaokai Lin
 */
public class InstructionBLT extends InstructionBranchBase {
  public InstructionBLT(Object rs1, Object rs2, Phase label) {
    super(rs1, rs2, label);
    this.opcode = Opcode.BLT;
  }
}
