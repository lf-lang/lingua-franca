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
  Register baseTime;

  /** The logical time to advance to */
  Register increment;

  /** Constructor */
  public InstructionADV(ReactorInstance reactor, Register baseTime, Register increment) {
    this.opcode = Opcode.ADV;
    this.reactor = reactor;
    this.baseTime = baseTime;
    this.increment = increment;
  }

  @Override
  public String toString() {
    return "ADV: " + "advance" + reactor + " to " + baseTime + " + " + increment;
  }

  @Override
  public Instruction clone() {
    return new InstructionADV(reactor, baseTime, increment);
  }
}
