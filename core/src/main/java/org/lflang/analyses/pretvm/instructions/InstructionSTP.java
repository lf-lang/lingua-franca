package org.lflang.analyses.pretvm.instructions;

/**
 * Class defining the STP instruction
 *
 * @author Shaokai Lin
 */
public class InstructionSTP extends Instruction<Object, Object, Object> {
  public InstructionSTP() {
    this.opcode = Opcode.STP;
  }

  @Override
  public Instruction<Object, Object, Object> clone() {
    return new InstructionSTP();
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionSTP) {
      return true;
    }
    return false;
  }

  @Override
  public String toString() {
    return "STP";
  }
}
