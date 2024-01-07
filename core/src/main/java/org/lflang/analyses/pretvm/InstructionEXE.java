package org.lflang.analyses.pretvm;

/**
 * Class defining the EXE instruction
 *
 * @author Shaokai Lin
 */
public class InstructionEXE extends Instruction {

  /** C function pointer to be executed */
  public String functionPointer;

  /** Constructor */
  public InstructionEXE(String functionPointer) {
    this.opcode = Opcode.EXE;
    this.functionPointer = functionPointer;
  }

  @Override
  public String toString() {
    return opcode + ": " + this.functionPointer;
  }

  @Override
  public Instruction clone() {
    return new InstructionEXE(functionPointer);
  }
}
