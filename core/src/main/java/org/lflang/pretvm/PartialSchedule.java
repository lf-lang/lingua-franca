package org.lflang.pretvm;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.pretvm.dag.Dag;
import org.lflang.pretvm.instruction.Instruction;

/**
 * A partial schedule contains instructions for one phase of execution, the
 * state space diagram of that phase, the DAG, and references to
 * upstream/downstream partial schedules.
 *
 * @author Shaokai J. Lin
 */
public class PartialSchedule {

  /**
   * A list of list of instructions, where the inner list is a sequence of instructions
   * for a worker, and the outer list is a list of instruction sequences, one for each worker.
   */
  private List<List<Instruction>> instructions;

  /** The state space diagram contained in this object file */
  StateSpaceDiagram diagram;

  Dag dagParitioned;

  /** A list of upstream object files this object file depends on */
  List<PartialSchedule> upstreams = new ArrayList<>();

  /** A list of downstream object files that depend on this object file */
  List<PartialSchedule> downstreams = new ArrayList<>();

  /**
   * A map from a downstream object file to a transition guard. A transition
   * guard is a sequence of instructions encoding conditional branch. 
   * 
   * FIXME: All workers for now will evaluate the same guard. This is arguably
   * redundant work that needs to be optimized away.
   */
  Map<PartialSchedule, List<Instruction>> guardMap = new HashMap<>();

  /** Getter for content */
  public List<List<Instruction>> getInstructions() {
    return instructions;
  }

  public void setInstructions(List<List<Instruction>> updatedInstructions) {
    this.instructions = updatedInstructions;
  }

  /** Check if the fragment is cyclic. */
  public boolean isCyclic() {
    return diagram.isCyclic();
  }

  /** Diagram getter */
  public StateSpaceDiagram getDiagram() {
    return diagram;
  }

  public Dag getDag() {
    return dagParitioned;
  }

  /** Get state space diagram phase. */
  public ExecutionPhase getPhase() {
    return diagram.phase;
  }

  /** Upstream getter */
  public List<PartialSchedule> getUpstreams() {
    return upstreams;
  }

  /** Upstream getter */
  public List<PartialSchedule> getDownstreams() {
    return downstreams;
  }

  /** Add an upstream fragment */
  public void addUpstream(PartialSchedule upstream) {
    this.upstreams.add(upstream);
  }

  /** Add an downstream fragment with a guarded transition */
  public void addDownstream(PartialSchedule downstream, List<Instruction> guard) {
    this.downstreams.add(downstream);
    this.guardMap.put(downstream, guard);
  }

  /** Pretty printing instructions */
  public void display() {
    List<List<Instruction>> instructions = this.getInstructions();
    for (int i = 0; i < instructions.size(); i++) {
      List<Instruction> schedule = instructions.get(i);
      System.out.println("Worker " + i + ":");
      for (int j = 0; j < schedule.size(); j++) {
        System.out.println(schedule.get(j));
      }
    }
  }
}