package org.lflang.analyses.pretvm;

import java.util.Objects;

/**
 * Class defining the JALR instruction
 *
 * @author Shaokai Lin
 */
public class InstructionJALR extends Instruction<Register,Register,Long> {

  /** Constructor */
  public InstructionJALR(Register destination, Register baseAddr, Long immediate) {
    this.opcode = Opcode.JALR;
    this.operand1 = destination; // A destination register to return to
    this.operand2 = baseAddr; // A register containing the base address
    this.operand3 = immediate; // A immediate representing the address offset
  }

  @Override
  public String toString() {
    return "JALR: "
        + "store the return address in "
        + this.operand1
        + " and jump to "
        + this.operand2
        + " + "
        + this.operand3;
  }

  @Override
  public Instruction<Register,Register,Long> clone() {
    return new InstructionJALR(this.operand1, this.operand2, this.operand3);
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionJALR that) {
      if (Objects.equals(this.operand1, that.operand1)
        && Objects.equals(this.operand2, that.operand2)
        && Objects.equals(this.operand3, that.operand3)) {
        return true;
      }
    }
    return false;
  }
}
