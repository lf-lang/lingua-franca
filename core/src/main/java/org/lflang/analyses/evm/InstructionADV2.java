package org.lflang.analyses.evm;

import org.lflang.TimeValue;
import org.lflang.generator.ReactorInstance;

public class InstructionADV2 extends Instruction {

  /** The reactor whose logical time is to be advanced */
  ReactorInstance reactor;

  /** The logical time to advance to */
  TimeValue nextTime;

  public InstructionADV2(ReactorInstance reactor, TimeValue nextTime) {
    this.opcode = Opcode.ADV2;
    this.reactor = reactor;
    this.nextTime = nextTime;
  }

  @Override
  public String toString() {
    return "ADV2: " + "advance" + reactor + " to " + nextTime + " wrt the hyperperiod.";
  }
}
