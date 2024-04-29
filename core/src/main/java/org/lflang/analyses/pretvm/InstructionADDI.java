package org.lflang.analyses.pretvm;

/**
 * Class defining the ADDI instruction
 *
 * @author Shaokai Lin
 */
public class InstructionADDI extends Instruction {

  /** The target and source registers */
  Register target, source;

  /** The immediate to be added with the variable */
  Long immediate;

  public InstructionADDI(
      Register target,
      Register source,
      Long immediate) {
    this.opcode = Opcode.ADDI;
    this.target = target;
    this.source = source;
    this.immediate = immediate;
  }

  @Override
  public String toString() {
    return "Increment "
        + target
        + " by adding "
        + source
        + " and "
        + immediate
        + "LL";
  }

  @Override
  public Instruction clone() {
    return new InstructionADDI(target, source, immediate);
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionADDI that) {
      if (this.target == that.target
        && this.source == that.source
        && this.immediate == that.immediate) {
        return true;
      }
    }
    return false;
  }
}
