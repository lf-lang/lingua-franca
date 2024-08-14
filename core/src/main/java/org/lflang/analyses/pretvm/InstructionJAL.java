package org.lflang.analyses.pretvm;

/**
 * Class defining the JAL instruction
 *
 * @author Shaokai Lin
 */
public class InstructionJAL extends Instruction<Register,Object,Integer> {

  /** Constructor */
  public InstructionJAL(Register retAddr, Object targetLabel) {
    this.opcode = Opcode.JAL;
    this.operand1 = retAddr; // A register to store the return address
    this.operand2 = targetLabel; // A target label to jump to
  }
  
  public InstructionJAL(Register retAddr, Object targetLabel, Integer offset) {
    this.opcode = Opcode.JAL;
    this.operand1 = retAddr; // A register to store the return address
    this.operand2 = targetLabel; // A target label to jump to
    this.operand3 = offset; // An additional offset
  }

  @Override
  public String toString() {
    return "JAL: " + "store return address in " + this.operand1 + " and jump to " + this.operand2 + (this.operand3 == null ? "" : " + " + this.operand3);
  }

  @Override
  public Instruction<Register,Object,Integer> clone() {
    return new InstructionJAL(this.operand1, this.operand2, this.operand3);
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionJAL that) {
      if (this.opcode == that.opcode
        && this.operand1 == that.operand1
        && this.operand2 == that.operand2
        && this.operand3 == that.operand3) {
        return true;
      }
    }
    return false;
  }
}
