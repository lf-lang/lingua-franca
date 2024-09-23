package org.lflang.analyses.pretvm;

import java.util.Objects;
import java.util.UUID;

import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;

/**
 * A memory label of an instruction, similar to a label in x86, arm, and RISC-V.
 *
 * @author Shaokai Lin
 */
public class PretVmLabel {
  
  /** Different types of label */
  public enum LabelType {
    ADVANCE_TAG,
    DELAY_INSTANTIATE,
    EXECUTE,
    PHASE,
    PROCEDURE,
    PROCESS_CONNECTION,
    TEST_TRIGGER,
  }

  /** Pointer to an instruction */
  Instruction instruction;

  /** The type of PretVM Label */
  LabelType type;

  /** 
   * A postfix string that contains useful info,
   * such as which reactor, which reaction, which port
   * this is for.
   */
  String postfix;

  /** Unique ID */
  String uuid;

  /** Constructor */
  public PretVmLabel(Instruction instruction, LabelType type, String postfix) {
    this.instruction = instruction;
    this.type = type;
    this.postfix = postfix;
    this.uuid = generateShortUUID();
  }

  /** Getter for the instruction */
  public Instruction getInstruction() {
    return instruction;
  }

  /** 
   * Return a string representation of the label to be embedded
   * in the C code. If the label is of type PHASE, do not
   * attach a UUID for the ease of reference, and there should be
   * only one PHASE label for phase.
   */
  @Override
  public String toString() {
    if (this.type == LabelType.PHASE)
      return this.type + "_" + this.postfix;
    return this.type + "_" + this.postfix + "_" + this.uuid;
  }

  /**
   * When two labels are compared for equality, only their types
   * and postfixes are checked, not the UUIDs, so that the DAG-based
   * optimizer can factor out two instructions jumping to the same
   * _class_ of labels. The labels do not have to be exactly the same.
   */
  @Override
  public boolean equals(Object label) {
    if (label instanceof PretVmLabel that) {
      if (Objects.equals(this.type, that.type)
        && Objects.equals(this.postfix, that.postfix)) {
        return true;
      }
    }
    return false;
  }

  /** Generate short UUID to guarantee uniqueness in strings */
  private String generateShortUUID() {
    return UUID.randomUUID().toString().substring(0, 8); // take first 8 characters
  }

  /** 
   * Prepend a PHASE label type to form a proper phase label.
   * This is useful for jumping to a phase label from anywhere
   * as long as the target phase is known.
   */
  public static String getPhaseLabel(Phase phase) {
    return PretVmLabel.LabelType.PHASE + "_" + phase;
  }

  public static String getProcedureLabel(Phase phase, Integer procedureIndex) {
    return PretVmLabel.LabelType.PROCEDURE + "_" + phase + "_" + procedureIndex;
  }

  /** Return a string of a label for a worker */
  public static String getWorkerLabelString(Object label, int worker) {
    if ((label instanceof PretVmLabel) || (label instanceof Phase) || (label instanceof String))
      return "WORKER" + "_" + worker + "_" + label.toString();
    throw new RuntimeException("Unsupported label type. Received: " + label.getClass().getName() + " = " + label);
  }
}
