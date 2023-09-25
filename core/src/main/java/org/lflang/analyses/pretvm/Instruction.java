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
   * <p>ADD rs1, rs2, rs3 : Add to an integer variable (rs2) by an integer variable (rs3) and store
   * the result in a destination variable (rs1).
   *
   * <p>ADDI rs1, rs2, rs3 : Add to an integer variable (rs2) by an immediate (rs3) and store the
   * result in a destination variable (rs1).
   *
   * <p>ADV rs1, rs2, rs3 : ADVance the logical time of a reactor (rs1) to a base time register
   * (rs2) + an increment register (rs3).
   *
   * <p>ADVI rs1, rs2, rs3 : Advance the logical time of a reactor (rs1) to a base time register
   * (rs2) + an immediate value (rs3). The compiler needs to guarantee only a single thread can
   * update a reactor's tag.
   *
   * <p>BEQ rs1, rs2, rs3 : Take the branch (rs3) if rs1 is equal to rs2.
   *
   * <p>BGE rs1, rs2, rs3 : Take the branch (rs3) if rs1 is greater than or equal to rs2.
   *
   * <p>BIT rs1 : (Branch-If-Timeout) Branch to a location (rs1) if all reactors reach timeout.
   *
   * <p>BLT rs1, rs2, rs3 : Take the branch (rs3) if rs1 is less than rs2.
   *
   * <p>BNE rs1, rs2, rs3 : Take the branch (rs3) if rs1 is not equal to rs2.
   *
   * <p>DU rs1, rs2 : Delay Until a physical timepoint (rs1) plus an offset (rs2) is reached.
   *
   * <p>EIT rs1 : Execute a reaction (rs1) If Triggered. FIXME: Combine with a branch.
   *
   * <p>EXE rs1 : EXEcute a reaction (rs1) (used for known triggers such as startup, shutdown, and
   * timers).
   *
   * <p>JAL rs1 rs2 : Store the return address to rs1 and jump to a label (rs2).
   *
   * <p>JALR rs1, rs2, rs3 : Store the return address in destination (rs1) and jump to baseAddr (rs2) + immediate
   * (rs3)
   *
   * <p>SAC (to remove) : (Sync-Advance-Clear) synchronize all workers until all execute SAC, advance logical
   * time to rs1, and let the last idle worker reset all counters to 0.
   *
   * <p>STP : SToP the execution.
   *
   * <p>WLT rs1, rs2 : Wait until a variable (rs1) owned by a worker (rs2) to be less than a desired
   * value (rs3).
   *
   * <p>WU rs1, rs2 : Wait Until a variable (rs1) owned by a worker (rs2) to be greater than or
   * equal to a desired value (rs3).
   */
  public enum Opcode {
    ADD,
    ADDI,
    ADV,
    ADVI,
    BEQ,
    BGE,
    BIT,
    BLT,
    BNE,
    DU,
    EIT,
    EXE,
    JAL,
    JALR,
    SAC,
    STP,
    WLT,
    WU,
  }

  /** Opcode of this instruction */
  protected Opcode opcode;

  /** A memory label for this instruction */
  private PretVmLabel label;

  // Copy Constructor
  // public Instruction(Instruction other) {
  //   this.opcode = other.opcode;
  //   if (other.label != null) {
  //     this.label = new PretVmLabel(this, other.label.toString());
  //   } else {
  //     this.label = null;
  //   }
  // }

  /** Getter of the opcode */
  public Opcode getOpcode() {
    return this.opcode;
  }

  /** Create a label for this instruction. */
  public void createLabel(String label) {
    // try {
    //   throw new Exception(label);
    // } catch (Exception e) {
    //   e.printStackTrace();
    // }
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
