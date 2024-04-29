package org.lflang.analyses.pretvm;

import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;

/**
 * A base class for branch instructions. According to the RISC-V specifications, the operands can
 * only be registers.
 *
 * @author Shaokai Lin
 */
public abstract class InstructionBranchBase extends Instruction {

  /** The first operand, either Register or String. */
  Register rs1;

  /** The second operand, either Register or String. */
  Register rs2;

  /**
   * The label to jump to, which can only be one of the phases (INIT, PERIODIC,
   * etc.) or a PretVmLabel. It cannot just be a number because numbers are hard
   * to be absolute before linking. It is recommended to use PretVmLabel objects.
   */
  Object label;

  public InstructionBranchBase(Register rs1, Register rs2, Object label) {
    if ((rs1 instanceof Register)
      && (rs2 instanceof Register)
      && (label instanceof Phase || label instanceof PretVmLabel)) {
      this.rs1 = rs1;
      this.rs2 = rs2;
      this.label = label;
    }
    else throw new RuntimeException(
      "Operands must be either Register or String. Label must be either Phase or PretVmLabel. Operand 1: "
      + rs1.getClass().getName() + ". Operand 2: " + rs2.getClass().getName() + ". Label: " + label.getClass().getName());
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionBranchBase that) {
      if (this.opcode == that.opcode
        && this.rs1 == that.rs1
        && this.rs2 == that.rs2
        && this.label == that.label) {
        return true;
      }
    }
    return false;
  }
}
