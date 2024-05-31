package org.lflang.analyses.pretvm;

/**
 * A memory label of an instruction, similar to the one in RISC-V
 *
 * @author Shaokai Lin
 */
public class PretVmLabel {
  /** Pointer to an instruction */
  Instruction instruction;

  /** A string label */
  String labelString;

  /** Constructor */
  public PretVmLabel(Instruction instruction, String labelString) {
    this.instruction = instruction;
    this.labelString = labelString;
  }

  /** Getter for the instruction */
  public Instruction getInstruction() {
    return instruction;
  }

  @Override
  public String toString() {
    return labelString;
  }
}
