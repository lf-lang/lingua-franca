package org.lflang.analyses.pretvm;

/**
 * Class defining the JAL instruction
 *
 * @author Shaokai Lin
 */
public class InstructionJAL extends Instruction {

  /** A register to store the return address */
  GlobalVarType retAddr;

  /** A target label to jump to */
  Object targetLabel;

  /** Constructor */
  public InstructionJAL(GlobalVarType destination, Object targetLabel) {
    this.opcode = Opcode.JAL;
    this.retAddr = destination;
    this.targetLabel = targetLabel;
  }

  @Override
  public String toString() {
    return "JAL: " + "store return address in " + retAddr + " and jump to " + targetLabel;
  }

  @Override
  public Instruction clone() {
    return new InstructionJAL(retAddr, targetLabel);
  }
}
