package org.lflang.analyses.pretvm;

import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;

/**
 * Class defining the JMP instruction
 *
 * @author Shaokai Lin
 */
public class InstructionJMP extends Instruction {

  /** A target phase to jump to */
  Phase target;

  /** Constructor */
  public InstructionJMP(Phase target) {
    this.opcode = Opcode.JMP;
    this.target = target;
  }

  @Override
  public String toString() {
    return "JMP: " + target;
  }
}
