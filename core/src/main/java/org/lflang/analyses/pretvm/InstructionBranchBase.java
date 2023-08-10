package org.lflang.analyses.pretvm;

import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;

/**
 * A base class for branch instructions. According to the RISC-V specifications, the operands can
 * only be registers.
 *
 * @author Shaokai Lin
 */
public class InstructionBranchBase extends Instruction {

  /** The first operand. This can either be a VarRef or a Long (i.e., an immediate). */
  GlobalVarType rs1;

  /** The second operand. This can either be a VarRef or a Long (i.e., an immediate). */
  GlobalVarType rs2;

  /**
   * The phase to jump to, which can only be one of the state space phases. This will be directly
   * converted to a label when generating C code.
   */
  Phase phase;

  public InstructionBranchBase(GlobalVarType rs1, GlobalVarType rs2, Phase phase) {
    if (phase == null) throw new RuntimeException("phase is null.");
    this.rs1 = rs1;
    this.rs2 = rs2;
    this.phase = phase;
  }
}
