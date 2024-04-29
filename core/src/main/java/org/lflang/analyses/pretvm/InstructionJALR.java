package org.lflang.analyses.pretvm;

/**
 * Class defining the JALR instruction
 *
 * @author Shaokai Lin
 */
public class InstructionJALR extends Instruction {

  /** A destination register to return to */
  Register destination;

  /** A register containing the base address */
  Register baseAddr;

  /** A immediate representing the address offset */
  Long immediate;

  /** Constructor */
  public InstructionJALR(Register destination, Register baseAddr, Long immediate) {
    this.opcode = Opcode.JALR;
    this.destination = destination;
    this.baseAddr = baseAddr;
    this.immediate = immediate;
  }

  @Override
  public String toString() {
    return "JALR: "
        + "store the return address in "
        + destination
        + " and jump to "
        + baseAddr
        + " + "
        + immediate;
  }

  @Override
  public Instruction clone() {
    return new InstructionJALR(destination, baseAddr, immediate);
  }

  @Override
  public boolean equals(Object inst) {
    if (inst instanceof InstructionJALR that) {
      if (this.destination == that.destination
        && this.baseAddr == that.baseAddr
        && this.immediate == that.immediate) {
        return true;
      }
    }
    return false;
  }
}
