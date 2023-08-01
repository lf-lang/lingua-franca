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
}
