package org.lflang.analyses.pretvm;

/**
 * Class defining the EXE instruction
 *
 * @author Shaokai Lin
 */
public class InstructionEXE extends Instruction {

  /** C function pointer to be executed */
  public String functionPointer;

  /** A pointer to an argument struct */
  public String functionArgumentPointer;

  /** 
   * A reaction number if this EXE executes a reaction. Null if the EXE executes
   * a helper function.
   */
  public Integer reactionNumber;

  /** Constructor */
  public InstructionEXE(String functionPointer, String functionArgumentPointer, Integer reactionNumber) {
    this.opcode = Opcode.EXE;
    this.functionPointer = functionPointer;
    this.functionArgumentPointer = functionArgumentPointer;
    this.reactionNumber = reactionNumber;
  }

  @Override
  public String toString() {
    return opcode + ": " + this.functionPointer;
  }

  @Override
  public Instruction clone() {
    return new InstructionEXE(functionPointer, functionArgumentPointer, reactionNumber);
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionEXE that) {
      if (this.functionPointer == that.functionPointer
        && this.functionArgumentPointer == that.functionArgumentPointer
        && this.reactionNumber == that.reactionNumber) {
        return true;
      }
    }
    return false;
  }
}
