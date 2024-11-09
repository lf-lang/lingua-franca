package org.lflang.analyses.pretvm.instructions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.lflang.analyses.dag.DagNode;
import org.lflang.analyses.pretvm.PretVmLabel;

/**
 * Abstract class defining a PRET virtual machine instruction
 *
 * @author Shaokai Lin
 */
public abstract class Instruction<T1, T2, T3> {

  /**
   * PRET VM Instruction Set
   *
   * <p>ADD rs1, rs2, rs3 : Add to an integer variable (rs2) by an integer variable (rs3) and store
   * the result in a destination variable (rs1).
   *
   * <p>ADDI rs1, rs2, rs3 : Add to an integer variable (rs2) by an immediate (rs3) and store the
   * result in a destination variable (rs1).
   *
   * <p>BEQ rs1, rs2, rs3 : Take the branch (rs3) if rs1 is equal to rs2.
   *
   * <p>BGE rs1, rs2, rs3 : Take the branch (rs3) if rs1 is greater than or equal to rs2.
   *
   * <p>BLT rs1, rs2, rs3 : Take the branch (rs3) if rs1 is less than rs2.
   *
   * <p>BNE rs1, rs2, rs3 : Take the branch (rs3) if rs1 is not equal to rs2.
   *
   * <p>DU rs1, rs2 : Delay Until a physical timepoint (rs1) plus an offset (rs2) is reached.
   *
   * <p>EXE rs1 : EXEcute a reaction (rs1) (used for known triggers such as startup, shutdown, and
   * timers).
   *
   * <p>JAL rs1 rs2 : Store the return address to rs1 and jump to a label (rs2).
   *
   * <p>JALR rs1, rs2, rs3 : Store the return address in destination (rs1) and jump to baseAddr
   * (rs2) + immediate (rs3)
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
    BEQ,
    BGE,
    BLT,
    BNE,
    DU,
    EXE,
    JAL,
    JALR,
    STP,
    WLT,
    WU,
  }

  /** Opcode of this instruction */
  protected Opcode opcode;

  /** The first operand */
  protected T1 operand1;

  /** The second operand */
  protected T2 operand2;

  /** The third operand */
  protected T3 operand3;

  /**
   * A list of memory label for this instruction. A line of code can have multiple labels, similar
   * to C.
   */
  private List<PretVmLabel> label;

  /** Worker who owns this instruction */
  private int worker;

  /** DAG node for which this instruction is generated */
  private DagNode node;

  /** Getter of the opcode */
  public Opcode getOpcode() {
    return this.opcode;
  }

  /** Set a label for this instruction. */
  public void addLabel(String labelString) {
    if (this.label == null)
      this.label = new ArrayList<>(Arrays.asList(new PretVmLabel(this, labelString)));
    else
      // If the list is already instantiated,
      // create a new label and add it to the list.
      this.label.add(new PretVmLabel(this, labelString));
  }

  /** Add a list of labels */
  public void addLabels(List<PretVmLabel> labels) {
    if (this.label == null) this.label = new ArrayList<>();
    this.label.addAll(labels);
  }

  /** Remove a label for this instruction. */
  public void removeLabel(PretVmLabel label) {
    this.label.remove(label);
  }

  /** Return true if the instruction has a label. */
  public boolean hasLabel() {
    return this.label != null;
  }

  /** Return the first label. */
  public PretVmLabel getLabel() {
    if (this.label.isEmpty()) return null;
    return this.label.get(0); // Get the first label by default.
  }

  /** Return the entire label list. */
  public List<PretVmLabel> getLabelList() {
    return this.label;
  }

  public int getWorker() {
    return this.worker;
  }

  public void setWorker(int worker) {
    this.worker = worker;
  }

  public DagNode getDagNode() {
    return this.node;
  }

  public void setDagNode(DagNode node) {
    this.node = node;
  }

  @Override
  public String toString() {
    return opcode.toString()
        + " "
        + operand1.toString()
        + " "
        + operand2.toString()
        + " "
        + operand3.toString();
  }

  public T1 getOperand1() {
    return this.operand1;
  }

  public void setOperand1(T1 operand) {
    this.operand1 = operand;
  }

  public T2 getOperand2() {
    return this.operand2;
  }

  public void setOperand2(T2 operand) {
    this.operand2 = operand;
  }

  public T3 getOperand3() {
    return this.operand3;
  }

  public void setOperand3(T3 operand) {
    this.operand3 = operand;
  }

  public List<Object> getOperands() {
    return Arrays.asList(operand1, operand2, operand3);
  }

  @Override
  public Instruction clone() {
    throw new UnsupportedOperationException("Unimplemented method 'clone'");
  }

  @Override
  public boolean equals(Object inst) {
    throw new UnsupportedOperationException("Unimplemented method 'clone'");
  }
}
