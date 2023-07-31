package org.lflang.analyses.pretvm;

import org.lflang.TimeValue;

public class InstructionSAC extends Instruction {

  /** The logical time to advance to */
  TimeValue nextTime;

  public InstructionSAC(TimeValue timeStep) {
    this.opcode = Opcode.SAC;
    this.nextTime = timeStep;
  }
}
