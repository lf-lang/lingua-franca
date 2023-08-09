package org.lflang.analyses.pretvm;

import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;

/**
 * A base class for branch instructions
 *
 * @author Shaokai Lin
 */
public class InstructionBranchBase extends Instruction {

  /** The first operand. This can either be a VarRef or a Long (i.e., an immediate). */
  Object rs1;

  /** The second operand. This can either be a VarRef or a Long (i.e., an immediate). */
  Object rs2;

  /**
   * The phase to jump to, which can only be one of the state space phases. This will be directly
   * converted to a label when generating C code.
   */
  Phase phase;

  public InstructionBranchBase(Object rs1, Object rs2, Phase phase) {
    if (!(rs1 instanceof GlobalVarType || rs1 instanceof Long)
        || !(rs2 instanceof GlobalVarType || rs2 instanceof Long))
      throw new RuntimeException("Invalid type found.");
    if (phase == null) throw new RuntimeException("phase is null.");
    this.rs1 = rs1;
    this.rs2 = rs2;
    this.phase = phase;
  }
}
