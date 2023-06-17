package org.lflang.analyses.evm;

import org.lflang.TimeValue;

public class InstructionADV2 extends Instruction {

  /** The logical time to advance to */
  TimeValue nextTime;

  public InstructionADV2(TimeValue nextTime) {
    this.opcode = Opcode.DU;
    this.nextTime = nextTime;
  }

  @Override
  public String toString() {
    return "ADV2: " + nextTime;
  }
}
