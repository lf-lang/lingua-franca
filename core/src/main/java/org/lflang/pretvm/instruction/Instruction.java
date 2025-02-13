package org.lflang.pretvm.instruction;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import org.lflang.pretvm.Label;
import org.lflang.pretvm.dag.DagNode;

/**
 * Abstract class defining a PretVM instruction
 *
 * @author Shaokai J. Lin
 */
public abstract class Instruction<T1, T2, T3> {

  //////////////////////////////////////////////////////////////////////
  /// Protected Variables

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
  private List<DagNode> nodes = new ArrayList<>();

  //////////////////////////////////////////////////////////////////////
  /// Public Methods

  /** Set a label for this instruction. */
  public void addLabel(String labelString) {
    if (this.label == null) {
      this.label = new ArrayList<>(Arrays.asList(new Label(labelString)));
    } else {
      // If the list is already instantiated,
      // create a new label and add it to the list.
      this.label.add(new Label(labelString));
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
  public void addNode(DagNode node) {
    this.nodes.add(node);
  }

  @Override
  public boolean equals(Object o) {
    if (o instanceof Instruction that
        // Check if instructions are the same.
        && this.getClass() == that.getClass()
        // Check if operands are the same.
        && (Objects.equals(this.operand1, that.operand1)
            && Objects.equals(this.operand2, that.operand2)
            && Objects.equals(this.operand3, that.operand3))) {
      return true;
    }
    return false;
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
  public DagNode getNode() {
    if (this.nodes.size() > 1)
      throw new RuntimeException("This instruction is generated for more than one node!");
    return this.nodes.get(0);
  }

  /**
   * Get a list of DAG nodes for which this instruction is generated.
   *
   * @return a list of DAG nodes
   */
  public List<DagNode> getNodes() {
    return this.nodes;
  }

  /**
   * Return the opcode of the instruction, which is the class name.
   *
   * @return the opcode of the instruction
   */
  public String getOpcode() {
    return this.getClass().getSimpleName();
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
    return this.getClass()
        + " "
        + operand1.toString()
        + " "
        + operand2.toString()
        + " "
        + operand3.toString();
  }
}
