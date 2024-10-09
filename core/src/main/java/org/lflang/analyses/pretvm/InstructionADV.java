package org.lflang.analyses.pretvm;

import java.util.Objects;

import org.lflang.generator.ReactorInstance;

/**
 * Class defining the ADV instruction
 *
 * @author Shaokai Lin
 */
public class InstructionADV extends Instruction<ReactorInstance,Register,Register> {

  /** Constructor */
  public InstructionADV(ReactorInstance reactor, Register baseTime, Register increment) {
    this.opcode = Opcode.ADV;
    this.operand1 = reactor; // The reactor whose logical time is to be advanced
    // A base variable upon which to apply the increment. This is usually the current time offset
    // (i.e., current time after applying multiple iterations of hyperperiods)
    this.operand2 = baseTime;
    this.operand3 = increment; // The logical time increment to add to the bast time
  }

  @Override
  public String toString() {
    return "ADV: " + "advance" + this.operand1 + " to " + this.operand2 + " + " + this.operand3;
  }

  @Override
  public Instruction<ReactorInstance,Register,Register> clone() {
    return new InstructionADV(this.operand1, this.operand2, this.operand3);
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionADV that) {
      if (Objects.equals(this.operand1, that.operand1)
        && Objects.equals(this.operand2, that.operand2)
        && Objects.equals(this.operand3, that.operand3)) {
        return true;
      }
    }
    return false;
  }
}
