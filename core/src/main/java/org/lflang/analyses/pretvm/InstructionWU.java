package org.lflang.analyses.pretvm;

/**
 * Class defining the WU instruction
 *
 * @author Shaokai Lin
 */
public class InstructionWU extends Instruction {

  /** A register WU waits on */
  public Register register;

  /** The value of a progress counter at which WU stops blocking */
  public Long releaseValue;

  public InstructionWU(Register register, Long releaseValue) {
    this.opcode = Opcode.WU;
    this.register = register;
    this.releaseValue = releaseValue;
  }

  @Override
  public String toString() {
    return "WU: Wait for " + register + " to reach " + releaseValue;
  }

  @Override
  public Instruction clone() {
    return new InstructionWU(register, releaseValue);
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionWU that) {
      if (this.register == that.register
        && this.releaseValue == that.releaseValue) {
        return true;
      }
    }
    return false;
  }
}
