package org.lflang.analyses.pretvm;

/**
 * Class defining the ADDI instruction
 *
 * @author Shaokai Lin
 */
public class InstructionADDI extends Instruction {

  /** Variable to be incremented */
  GlobalVarType target;

  /** The variable to be added with the immediate */
  GlobalVarType source;

  /** The immediate to be added with the variable */
  Long immediate;

  public InstructionADDI(GlobalVarType target, GlobalVarType source, Long immediate) {
    this.opcode = Opcode.ADDI;
    this.target = target;
    this.source = source;
    this.immediate = immediate;
  }
}
