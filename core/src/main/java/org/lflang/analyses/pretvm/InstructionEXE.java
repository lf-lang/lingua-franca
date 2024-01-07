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

  /** Constructor */
  public InstructionEXE(String functionPointer, String functionArgumentPointer) {
    this.opcode = Opcode.EXE;
    this.functionPointer = functionPointer;
    this.functionArgumentPointer = functionArgumentPointer;
  }

  @Override
  public String toString() {
    return opcode + ": " + this.functionPointer;
  }

  @Override
  public Instruction clone() {
    return new InstructionEXE(functionPointer, functionArgumentPointer);
  }
}
