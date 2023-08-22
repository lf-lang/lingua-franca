package org.lflang.analyses.pretvm;

/**
 * Class defining the ADD instruction
 *
 * @author Shaokai Lin
 */
public class InstructionADD extends Instruction {

  /** Variable to be incremented */
  GlobalVarType target;

  /** Worker who owns the target variable */
  Integer targetOwner;

  /** Variables to be added together */
  GlobalVarType source, source2;

  /** Workers who own the source variables */
  Integer sourceOwner, source2Owner;

  public InstructionADD(
      GlobalVarType target,
      Integer targetOwner,
      GlobalVarType source,
      Integer sourceOwner,
      GlobalVarType source2,
      Integer source2Owner) {
    this.opcode = Opcode.ADD;
    this.target = target;
    this.targetOwner = targetOwner;
    this.source = source;
    this.sourceOwner = sourceOwner;
    this.source2 = source2;
    this.source2Owner = source2Owner;
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
        + (source2Owner == null ? "" : "worker " + source2Owner + "'s ")
        + source2;
  }
}
