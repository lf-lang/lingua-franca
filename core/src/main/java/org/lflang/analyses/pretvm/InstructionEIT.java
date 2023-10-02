package org.lflang.analyses.pretvm;

import org.lflang.generator.ReactionInstance;

/**
 * Class defining the EIT instruction
 *
 * @author Shaokai Lin
 */
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

  @Override
  public Instruction clone() {
    return new InstructionEIT(reaction);
  }
}
