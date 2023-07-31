package org.lflang.analyses.pretvm;

public class InstructionWU extends Instruction {

  /** The value of the counting lock at which WU stops blocking */
  int releaseValue;

  /** ID of the worker owning the counting lock */
  int worker;

  public InstructionWU(int worker, int releaseValue) {
    this.opcode = Opcode.WU;
    this.releaseValue = releaseValue;
    this.worker = worker;
  }

  @Override
  public String toString() {
    return "WU: wait until worker " + worker + "'s counting lock reaches " + releaseValue;
  }
}
