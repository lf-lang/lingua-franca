package org.lflang.analyses.pretvm;

import org.lflang.TimeValue;

/**
 * Class defining the DU instruction
 *
 * @author Shaokai Lin
 */
public class InstructionDU extends Instruction {

  /** The physical time point to delay until */
  TimeValue releaseTime;

  public InstructionDU(TimeValue releaseTime) {
    this.opcode = Opcode.DU;
    this.releaseTime = releaseTime;
  }

  @Override
  public String toString() {
    return "DU: " + releaseTime;
  }

  @Override
  public Instruction clone() {
    return new InstructionDU(releaseTime);
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionDU that) {
      if (this.releaseTime == that.releaseTime) {
        return true;
      }
    }
    return false;
  }
}
