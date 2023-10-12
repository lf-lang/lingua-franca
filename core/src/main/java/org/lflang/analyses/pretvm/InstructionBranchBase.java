package org.lflang.analyses.pretvm;

import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;

/**
 * A base class for branch instructions. According to the RISC-V specifications, the operands can
 * only be registers.
 *
 * @author Shaokai Lin
 */
public abstract class InstructionBranchBase extends Instruction {

  /** The first operand, either GlobalVarType or String. */
  Object rs1;

  /** The second operand, either GlobalVarType or String. */
  Object rs2;

  /**
   * The label to jump to, which can only be one of the phases (INIT, PERIODIC,
   * etc.) or a PretVmLabel. It cannot just be a number because numbers are hard
   * to be absolute before linking. It is recommended to use PretVmLabel objects.
   */
  Object label;

  public InstructionBranchBase(Object rs1, Object rs2, Object label) {
    if ((rs1 instanceof GlobalVarType || rs1 instanceof String)
      && (rs2 instanceof GlobalVarType || rs2 instanceof String)
      && (label instanceof Phase || label instanceof PretVmLabel)) {
      this.rs1 = rs1;
      this.rs2 = rs2;
      this.label = label;
    }
    else throw new RuntimeException("An operand must be either GlobalVarType or String.");
  }
}
