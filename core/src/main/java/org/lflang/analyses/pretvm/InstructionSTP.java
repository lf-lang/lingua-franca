package org.lflang.analyses.pretvm;

/**
 * Class defining the STP instruction
 *
 * @author Shaokai Lin
 */
public class InstructionSTP extends Instruction {
  public InstructionSTP() {
    this.opcode = Opcode.STP;
  }
  @Override
  public Instruction clone() {
    return new InstructionSTP();
  }
}
