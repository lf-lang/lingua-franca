package org.lflang.analyses.pretvm;

/**
 * Class defining the JMP instruction
 *
 * @author Shaokai Lin
 */
public class InstructionJMP extends Instruction {

  /** The instruction to jump to */
  Instruction target;

  /** Constructor */
  public InstructionJMP(Instruction target) {
    this.opcode = Opcode.JMP;
    this.target = target;
  }

  @Override
  public String toString() {
    return "JMP: " + target;
  }
}
