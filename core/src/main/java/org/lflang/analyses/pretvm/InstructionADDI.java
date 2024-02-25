package org.lflang.analyses.pretvm;

/**
 * Class defining the ADDI instruction
 *
 * @author Shaokai Lin
 */
public class InstructionADDI extends Instruction {

  /** Variable to be incremented */
  GlobalVarType target;

  /** Worker who owns the target variable */
  Integer targetOwner;

  /** The variable to be added with the immediate */
  GlobalVarType source;

  /** Worker who owns the source variable */
  Integer sourceOwner;

  /** The immediate to be added with the variable */
  Long immediate;

  public InstructionADDI(
      GlobalVarType target,
      Integer targetOwner,
      GlobalVarType source,
      Integer sourceOwner,
      Long immediate) {
    this.opcode = Opcode.ADDI;
    this.target = target;
    this.targetOwner = targetOwner;
    this.source = source;
    this.sourceOwner = sourceOwner;
    this.immediate = immediate;
  }

  @Override
  public String toString() {
    return "Increment "
        + (targetOwner == null ? "" : "worker " + targetOwner + "'s ")
        + target
        + " by adding "
        + (sourceOwner == null ? "" : "worker " + sourceOwner + "'s ")
        + source
        + " and "
        + immediate
        + "LL";
  }

  @Override
  public Instruction clone() {
    return new InstructionADDI(target, targetOwner, source, sourceOwner, immediate);
  }
}
