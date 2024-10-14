package org.lflang.analyses.pretvm.instructions;

import java.util.Objects;

import org.lflang.analyses.pretvm.Register;

/**
 * Class defining the EXE instruction
 *
 * @author Shaokai Lin
 */
public class InstructionEXE extends Instruction<Register, Register, Integer> {

  /** Constructor */
  public InstructionEXE(
      Register functionPointer, Register functionArgumentPointer, Integer reactionNumber) {
    this.opcode = Opcode.EXE;
    this.operand1 = functionPointer; // C function pointer to be executed
    this.operand2 = functionArgumentPointer; // A pointer to an argument struct
    // A reaction number if this EXE executes a reaction. Null if the EXE executes
    // a helper function.
    this.operand3 = reactionNumber;
  }

  @Override
  public String toString() {
    return opcode + ": " + this.operand1 + " " + this.operand2 + " " + this.operand3;
  }

  @Override
  public Instruction<Register, Register, Integer> clone() {
    return new InstructionEXE(this.operand1, this.operand2, this.operand3);
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionEXE that) {
      if (Objects.equals(this.operand1, that.operand1)
          && Objects.equals(this.operand2, that.operand2)
          && Objects.equals(this.operand3, that.operand3)) {
        return true;
      }
    }
    return false;
  }
}
