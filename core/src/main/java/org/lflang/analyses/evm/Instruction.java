package org.lflang.analyses.evm;

public abstract class Instruction {

  /**
   * VM Instruction Set
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
   * <p>INC rs1, rs2 : INCrement a counter (rs1) by an amount (rs2).
   *
   * <p>INC2 rs1, rs2 : Lock-free version of INC. The compiler needs to guarantee single writer.
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
    ADV,
    ADV2,
    BIT,
    DU,
    EIT,
    EXE,
    INC,
    INC2,
    JMP,
    SAC,
    STP,
    WU,
  }

  /** Opcode of this instruction */
  protected Opcode opcode;

  /** A getter of the opcode */
  public Opcode getOpcode() {
    return this.opcode;
  }

  public String toString() {
    return opcode.toString();
  }
}
