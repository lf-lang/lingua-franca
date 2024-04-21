package org.lflang.analyses.pretvm;

/**
 * Class defining the WLT instruction
 *
 * @author Shaokai Lin
 */
public class InstructionWLT extends Instruction {

  /** A register WU waits on */
  Register register;

  /** The value of the register at which WU stops blocking */
  Long releaseValue;

  public InstructionWLT(Register register, Long releaseValue) {
    this.opcode = Opcode.WLT;
    this.register = register;
    this.releaseValue = releaseValue;
  }

  @Override
  public String toString() {
    return "WU: Wait for " + register + " to be less than " + releaseValue;
  }

  @Override
  public Instruction clone() {
    return new InstructionWLT(register, releaseValue);
  }
}
