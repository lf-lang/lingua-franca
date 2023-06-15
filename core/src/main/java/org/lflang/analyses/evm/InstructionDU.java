package org.lflang.analyses.evm;

import org.lflang.TimeValue;

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
