package org.lflang.analyses.pretvm;

/**
 * Class defining the WLT instruction
 *
 * @author Shaokai Lin
 */
public class InstructionWLT extends Instruction {

  /** A variable WU waits on */
  GlobalVarType variable;

  /** A worker who owns the variable */
  Integer owner;

  /** The value of the variable at which WU stops blocking */
  Long releaseValue;

  public InstructionWLT(GlobalVarType variable, Integer owner, Long releaseValue) {
    this.opcode = Opcode.WLT;
    this.variable = variable;
    this.owner = owner;
    this.releaseValue = releaseValue;
  }

  @Override
  public String toString() {
    return "WU: Wait for worker " + owner + "'s " + variable + " to be less than " + releaseValue;
  }

  @Override
  public Instruction clone() {
    return new InstructionWLT(variable, owner, releaseValue);
  }
}
