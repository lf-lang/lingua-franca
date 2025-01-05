package org.lflang.pretvm.instruction;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.lflang.pretvm.Label;
import org.lflang.pretvm.dag.Node;

/**
 * Abstract class defining a PretVM instruction
 *
 * @author Shaokai J. Lin
 */
public abstract class Instruction<T1, T2, T3> {

  /**
   * PretVM Instruction Set
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

  //////////////////////////////////////////////////////////////////////
  /// Protected Variables

  /** Opcode of this instruction */
  protected Opcode opcode;

  /** The first operand */
  protected T1 operand1;

  /** The second operand */
  protected T2 operand2;

  /** The third operand */
  protected T3 operand3;

  //////////////////////////////////////////////////////////////////////
  /// Private Variables

  /**
   * A list of memory label for this instruction. A line of code can have multiple labels, similar
   * to C.
   */
  private List<Label> label;

  /** Worker who owns this instruction */
  private int worker;

  /**
   * A list of DAG nodes for which this instruction is generated. This is a list because an
   * instruction can be generated for nodes in different phases. For example, a WU instruction in
   * the sync block.
   */
  private List<Node> nodes = new ArrayList<>();

  //////////////////////////////////////////////////////////////////////
  /// Public Methods

  /** Set a label for this instruction. */
  public void addLabel(String labelString) {
    if (this.label == null) {
      this.label = new ArrayList<>(Arrays.asList(new Label(this, labelString)));
    } else {
      // If the list is already instantiated,
      // create a new label and add it to the list.
      this.label.add(new Label(this, labelString));
    }
  }

  /** Add a list of labels */
  public void addLabels(List<Label> labels) {
    if (this.label == null) this.label = new ArrayList<>();
    this.label.addAll(labels);
  }

  /**
   * Add a DAG node for which this instruction is generated.
   *
   * @param node the DAG node for which this instruction is generated
   */
  public void addNode(Node node) {
    this.nodes.add(node);
  }

  /** Return the first label. */
  public Label getLabel() {
    if (this.label.isEmpty()) return null;
    return this.label.get(0); // Get the first label by default.
  }

  /** Return the entire list of labels. */
  public List<Label> getLabels() {
    return this.label;
  }

  /**
   * Get the DAG node for which this instruction is generated.
   *
   * @return a DAG node
   */
  public Node getNode() {
    if (this.nodes.size() > 1)
      throw new RuntimeException("This instruction is generated for more than one node!");
    return this.nodes.get(0);
  }

  /**
   * Get a list of DAG nodes for which this instruction is generated.
   *
   * @return a list of DAG nodes
   */
  public List<Node> getNodes() {
    return this.nodes;
  }

  /** Getter of the opcode */
  public Opcode getOpcode() {
    return this.opcode;
  }

  /**
   * Getter for the first operand
   *
   * @return the first operand
   */
  public T1 getOperand1() {
    return this.operand1;
  }

  /**
   * Getter for the second operand
   *
   * @return the second operand
   */
  public T2 getOperand2() {
    return this.operand2;
  }

  /**
   * Getter for the third operand
   *
   * @return the third operand
   */
  public T3 getOperand3() {
    return this.operand3;
  }

  /**
   * Get a list of operands
   *
   * @return the list containing all operands
   */
  public List<Object> getOperands() {
    return Arrays.asList(operand1, operand2, operand3);
  }

  /** Get the worker ID that owns this instruction. */
  public int getWorker() {
    return this.worker;
  }

  /** Return true if the instruction has a label. */
  public boolean hasLabel() {
    return this.label != null;
  }

  /** Remove a label for this instruction. */
  public void removeLabel(Label label) {
    this.label.remove(label);
  }

  /** Setter for operand 1 */
  public void setOperand1(T1 operand) {
    this.operand1 = operand;
  }

  /** Setter for operand 2 */
  public void setOperand2(T2 operand) {
    this.operand2 = operand;
  }

  /** Setter for operand 3 */
  public void setOperand3(T3 operand) {
    this.operand3 = operand;
  }

  /** Get a worker that owns this instruction. */
  public void setWorker(int worker) {
    this.worker = worker;
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
}
