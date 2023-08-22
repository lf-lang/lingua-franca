package org.lflang.analyses.pretvm;

import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;

/**
 * Class defining the JAL instruction
 *
 * @author Shaokai Lin
 */
public class InstructionJAL extends Instruction {

  /** A register to store the return address */
  GlobalVarType retAddr;

  /** A target phase to jump to */
  Phase target;

  /** Constructor */
  public InstructionJAL(GlobalVarType destination, Phase target) {
    this.opcode = Opcode.JAL;
    this.retAddr = destination;
    this.target = target;
  }

  @Override
  public String toString() {
    return "JAL: " + "store return address in " + retAddr + " and jump to " + target;
  }
}
