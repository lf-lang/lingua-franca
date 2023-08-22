package org.lflang.analyses.pretvm;

import org.lflang.generator.ReactorInstance;

/**
 * Class defining the ADV instruction
 *
 * @author Shaokai Lin
 */
public class InstructionADV extends Instruction {

  /** The reactor whose logical time is to be advanced */
  ReactorInstance reactor;

  /**
   * A base variable upon which to apply the increment. This is usually the current time offset
   * (i.e., current time after applying multiple iterations of hyperperiods)
   */
  GlobalVarType baseTime;

  /** The logical time to advance to */
  GlobalVarType increment;

  /** Constructor */
  public InstructionADV(ReactorInstance reactor, GlobalVarType baseTime, GlobalVarType increment) {
    this.opcode = Opcode.ADV;
    this.baseTime = baseTime;
    this.reactor = reactor;
    this.increment = increment;
  }

  @Override
  public String toString() {
    return "ADV: " + "advance" + reactor + " to " + baseTime + " + " + increment;
  }
}
