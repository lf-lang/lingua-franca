package org.lflang.pretvm;

/**
 * A memory label of an instruction, similar to labels in C assembly. The instructions should have
 * references to the labels, but the labels do not need to contain references to the instructions,
 * because when generating JAL instructions, we know which phase to jump to. But at that point,
 * concrete instructions that carry a label pointing to the phase have not been generated yet.
 *
 * @author Shaokai J. Lin
 */
public class Label {

  /** A string label */
  String labelString;

  /** Constructor */
  public Label(String labelString) {
    this.labelString = labelString;
  }

  /** Get a label from an execution phase */
  public static Label getExecutionPhaseLabel(ExecutionPhase phase) {
    return new Label(phase.toString());
  }

  /** Check if two labels are equal by checking the equality of their strings. */
  @Override
  public boolean equals(Object o) {
    if (this == o) return true;
    else if (o instanceof Label l) {
      return this.labelString.equals(l.labelString);
    }
    return false;
  }

  @Override
  public String toString() {
    return labelString;
  }
}
