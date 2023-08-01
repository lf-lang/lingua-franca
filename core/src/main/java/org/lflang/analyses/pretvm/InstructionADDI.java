package org.lflang.analyses.pretvm;

/**
 * Class defining the ADDI instruction
 *
 * @author Shaokai Lin
 */
public class InstructionADDI extends Instruction {

  /** Types of variables this instruction can update */
  public enum TargetVarType {
    OFFSET,
    COUNTER
  }

  /** Target variable */
  TargetVarType target;

  /** The value to be added */
  Long immediate;

  public InstructionADDI(TargetVarType target, Long immediate) {
    this.opcode = Opcode.ADDI;
    this.target = target;
    this.immediate = immediate;
  }
}
