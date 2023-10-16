package org.lflang.analyses.pretvm;

import org.lflang.generator.ReactorInstance;

/**
 * Class defining the ADVI (advance immediate) instruction
 *
 * @author Shaokai Lin
 */
public class InstructionADVI extends Instruction {

  /** The reactor whose logical time is to be advanced */
  ReactorInstance reactor;

  /**
   * A base variable upon which to apply the increment. This is usually the current time offset
   * (i.e., current time after applying multiple iterations of hyperperiods)
   */
  GlobalVarType baseTime;

  /** The logical time to advance to */
  Long increment;

  /** Constructor */
  public InstructionADVI(ReactorInstance reactor, GlobalVarType baseTime, Long increment) {
    this.opcode = Opcode.ADVI;
    this.baseTime = baseTime;
    this.increment = increment;
  }

  @Override
  public String toString() {
    return "ADVI: " + "advance " + reactor + " to " + baseTime + " + " + increment;
  }

  @Override
  public Instruction clone() {
    return new InstructionADVI(reactor, baseTime, increment);
  }
}
