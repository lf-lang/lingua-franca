package org.lflang.analyses.pretvm;

import org.lflang.generator.ReactionInstance;

/**
 * Class defining the EXE instruction
 *
 * @author Shaokai Lin
 */
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

  @Override
  public Instruction clone() {
    return new InstructionEXE(reaction);
  }
}
