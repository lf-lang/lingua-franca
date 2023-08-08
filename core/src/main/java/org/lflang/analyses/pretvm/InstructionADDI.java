package org.lflang.analyses.pretvm;

/**
 * Class defining the ADDI instruction
 *
 * @author Shaokai Lin
 */
public class InstructionADDI extends Instruction {

  /** Variable to be incremented */
  GlobalVarType target;

  /** The value to be added */
  Long immediate;

  public InstructionADDI(GlobalVarType target, Long immediate) {
    this.opcode = Opcode.ADDI;
    this.target = target;
    this.immediate = immediate;
  }
}
