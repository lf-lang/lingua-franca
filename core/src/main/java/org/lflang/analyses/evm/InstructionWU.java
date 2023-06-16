package org.lflang.analyses.evm;

import org.lflang.generator.ReactionInstance;

public class InstructionWU extends Instruction {

  /** The reaction this WU instruction waits on */
  ReactionInstance reaction;

  /** ID of the worker processing the reaction */
  int worker;

  public InstructionWU(int worker, ReactionInstance reaction) {
    this.opcode = Opcode.WU;
    this.worker = worker;
    this.reaction = reaction;
  }

  @Override
  public String toString() {
    return "WU: worker " + worker + " finish " + reaction;
  }
}
