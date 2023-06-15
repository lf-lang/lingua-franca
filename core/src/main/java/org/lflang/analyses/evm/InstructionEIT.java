package org.lflang.analyses.evm;

import org.lflang.generator.ReactionInstance;

public class InstructionEIT extends Instruction {

  /** Reaction to be executed */
  public ReactionInstance reaction;

  /** Constructor */
  public InstructionEIT(ReactionInstance reaction) {
    this.opcode = Opcode.EIT;
    this.reaction = reaction;
  }

  @Override
  public String toString() {
    return opcode + ": " + this.reaction;
  }
}
