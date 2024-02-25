package org.lflang.analyses.pretvm;

/**
 * Class defining the JALR instruction
 *
 * @author Shaokai Lin
 */
public class InstructionJALR extends Instruction {

  /** A destination register to return to */
  GlobalVarType destination;

  /** A register containing the base address */
  GlobalVarType baseAddr;

  /** A immediate representing the address offset */
  Long immediate;

  /** Constructor */
  public InstructionJALR(GlobalVarType destination, GlobalVarType baseAddr, Long immediate) {
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
}
