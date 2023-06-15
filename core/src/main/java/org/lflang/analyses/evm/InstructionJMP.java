package org.lflang.analyses.evm;

public class InstructionJMP extends Instruction {
  public InstructionJMP() {
    this.opcode = Opcode.JMP;
  }
}
