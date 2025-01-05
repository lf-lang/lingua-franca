package org.lflang.pretvm;

import org.lflang.pretvm.instruction.Instruction;

/**
 * A memory label of an instruction, similar to labels in C assembly.
 *
 * @author Shaokai J. Lin
 */
public class Label {
  /** Pointer to an instruction */
  Instruction instruction;

  /** A string label */
  String labelString;

  /** Constructor */
  public Label(Instruction instruction, String labelString) {
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
