package org.lflang.analyses.evm;

import org.lflang.generator.ReactionInstance;

public class InstructionEXE extends Instruction {

  /** Reaction to be executed */
  public ReactionInstance reaction;

  /** Constructor */
  public InstructionEXE(ReactionInstance reaction) {
    this.opcode = Opcode.EXE;
    this.reaction = reaction;
  }

  @Override
  public String toString() {
    return opcode + ": " + this.reaction;
  }
}
