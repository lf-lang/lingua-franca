package org.lflang.analyses.pretvm;

import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;

/**
 * A base class for branch instructions. According to the RISC-V specifications, the operands can
 * only be registers.
 *
 * @author Shaokai Lin
 */
public abstract class InstructionBranchBase extends Instruction<Register,Register,Object> {

  public InstructionBranchBase(Register rs1, Register rs2, Object label) {
    if ((rs1 instanceof Register)
      && (rs2 instanceof Register)
      && (label instanceof Phase || label instanceof PretVmLabel)) {
      this.operand1 = rs1; // The first operand, either Register or String
      this.operand2 = rs2; // The second operand, either Register or String
      // The label to jump to, which can only be one of the phases (INIT, PERIODIC,
      // etc.) or a PretVmLabel. It cannot just be a number because numbers are hard
      // to be absolute before linking. It is recommended to use PretVmLabel objects.
      this.operand3 = label;
    }
    else throw new RuntimeException(
      "Operands must be either Register or String. Label must be either Phase or PretVmLabel. Operand 1: "
      + rs1.getClass().getName() + ". Operand 2: " + rs2.getClass().getName() + ". Label: " + label.getClass().getName());
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionBranchBase that) {
      if (this.opcode == that.opcode
        && this.operand1 == that.operand1
        && this.operand2 == that.operand2
        && this.operand3 == that.operand3) {
        return true;
      }
    }
    return false;
  }
}
