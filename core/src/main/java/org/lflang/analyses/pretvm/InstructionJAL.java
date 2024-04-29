package org.lflang.analyses.pretvm;

/**
 * Class defining the JAL instruction
 *
 * @author Shaokai Lin
 */
public class InstructionJAL extends Instruction {

  /** A register to store the return address */
  Register retAddr;

  /** A target label to jump to */
  Object targetLabel;

  /** An additional offset */
  Integer offset;

  /** Constructor */
  public InstructionJAL(Register retAddr, Object targetLabel) {
    this.opcode = Opcode.JAL;
    this.retAddr = retAddr;
    this.targetLabel = targetLabel;
  }
  
  public InstructionJAL(Register retAddr, Object targetLabel, Integer offset) {
    this.opcode = Opcode.JAL;
    this.retAddr = retAddr;
    this.targetLabel = targetLabel;
    this.offset = offset;
  }

  @Override
  public String toString() {
    return "JAL: " + "store return address in " + retAddr + " and jump to " + targetLabel + (offset == null ? "" : " + " + offset);
  }

  @Override
  public Instruction clone() {
    return new InstructionJAL(retAddr, targetLabel, offset);
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionJAL that) {
      if (this.retAddr == that.retAddr
        && this.targetLabel == that.targetLabel
        && this.offset == that.offset) {
        return true;
      }
    }
    return false;
  }
}
