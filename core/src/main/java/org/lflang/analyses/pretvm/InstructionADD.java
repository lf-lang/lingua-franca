package org.lflang.analyses.pretvm;

/**
 * Class defining the ADD instruction
 *
 * @author Shaokai Lin
 */
public class InstructionADD extends Instruction {

  Register target, source, source2;

  public InstructionADD(
      Register target,
      Register source,
      Register source2
  ) {
    this.opcode = Opcode.ADD;
    this.target = target;
    this.source = source;
    this.source2 = source2;
  }

  @Override
  public String toString() {
    return "Increment "
        + target
        + " by adding "
        + source
        + " and "
        + source2;
  }

  @Override
  public Instruction clone() {
    return new InstructionADD(target, source, source2);
  }
}
