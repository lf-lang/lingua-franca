package org.lflang.analyses.pretvm;

/**
 * Class defining the BIT instruction
 *
 * @author Shaokai Lin
 */
public class InstructionBIT extends Instruction {
  public InstructionBIT() {
    this.opcode = Opcode.BIT;
  }
  @Override
  public Instruction clone() {
    return new InstructionBIT();
  }
}
