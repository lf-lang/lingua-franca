package org.lflang.analyses.pretvm;

/**
 * Abstract class defining a PRET virtual machine instruction
 *
 * @author Shaokai Lin
 */
public abstract class Instruction {

  /**
   * PRET VM Instruction Set
   *
   * <p>ADDI rs1, rs2, rs3 : Add to an integer variable (rs2) by an amount (rs3) and store the
   * result in a destination variable (rs1).
   *
   * <p>ADV rs1, rs2 : ADVance the logical time of a reactor (rs1) by a specified amount (rs2). Add
   * a delay_until here.
   *
   * <p>ADV2 rs1, rs2 : Lock-free version of ADV. The compiler needs to guarantee only a single
   * thread can update a reactor's tag.
   *
   * <p>BIT rs1, : (Branch-If-Timeout) Branch to a location (rs1) if all reactors reach timeout.
   *
   * <p>DU rs1, rs2 : Delay Until a physical timepoint (rs1) plus an offset (rs2) is reached.
   *
   * <p>EIT rs1 : Execute a reaction (rs1) If Triggered. FIXME: Combine with a branch.
   *
   * <p>EXE rs1 : EXEcute a reaction (rs1) (used for known triggers such as startup, shutdown, and
   * timers).
   *
   * <p>JMP rs1 : JuMP to a location (rs1).
   *
   * <p>SAC : (Sync-Advance-Clear) synchronize all workers until all execute SAC, advance logical
   * time to rs1, and let the last idle worker reset all counters to 0.
   *
   * <p>STP : SToP the execution.
   *
   * <p>WU rs1, rs2 : Wait Until a counting variable (rs1) to reach a desired value (rs2).
   */
  public enum Opcode {
    ADDI,
    ADV,
    ADV2,
    BIT,
    DU,
    EIT,
    EXE,
    JMP,
    SAC,
    STP,
    WU,
  }

  /** Opcode of this instruction */
  protected Opcode opcode;

  /** A memory label for this instruction */
  private PretVmLabel label;

  /** Getter of the opcode */
  public Opcode getOpcode() {
    return this.opcode;
  }

  /** Create a label for this instruction. */
  public void createLabel(String label) {
    this.label = new PretVmLabel(this, label);
  }

  /** Return true if the instruction has a label. */
  public boolean hasLabel() {
    return this.label != null;
  }

  /** Return the label. */
  public PretVmLabel getLabel() {
    return this.label;
  }

  @Override
  public String toString() {
    return opcode.toString();
  }
}
