package org.lflang.analyses.pretvm;

/**
 * Class defining the WU instruction
 *
 * @author Shaokai Lin
 */
public class InstructionWU extends Instruction {

  /** A variable WU waits on */
  GlobalVarType variable;

  /** A worker who owns the variable */
  Integer owner;

  /** The value of the variable at which WU stops blocking */
  Long releaseValue;

  public InstructionWU(GlobalVarType variable, Integer owner, Long releaseValue) {
    this.opcode = Opcode.WU;
    this.variable = variable;
    this.owner = owner;
    this.releaseValue = releaseValue;
  }

  @Override
  public String toString() {
    return "WU: Wait for worker " + owner + "'s " + variable + " to reach " + releaseValue;
  }

  @Override
  public Instruction clone() {
    return new InstructionWU(variable, owner, releaseValue);
  }
}
