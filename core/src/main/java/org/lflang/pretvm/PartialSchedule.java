package org.lflang.pretvm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.pretvm.dag.Dag;
import org.lflang.pretvm.instruction.BGE;
import org.lflang.pretvm.instruction.Instruction;
import org.lflang.pretvm.instruction.JAL;
import org.lflang.pretvm.register.ReturnAddr;

/**
 * A partial schedule contains instructions for one phase of execution, the state space diagram of
 * that phase, the DAG, and references to upstream/downstream partial schedules.
 *
 * @author Shaokai J. Lin
 */
public class PartialSchedule {

  /**
   * A list of list of instructions, where the inner list is a sequence of instructions for a
   * worker, and the outer list is a list of instruction sequences, one for each worker.
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
   * A map from a downstream object file to a transition guard. A transition guard is a sequence of
   * instructions encoding conditional branch.
   *
   * <p>FIXME: All workers for now will evaluate the same guard. This is arguably redundant work
   * that needs to be optimized away.
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

  /** Diagram getter */
  public void setDiagram(StateSpaceDiagram ssd) {
    diagram = ssd;
  }

  public Dag getDag() {
    return dagParitioned;
  }

  public void setDag(Dag dag) {
    dagParitioned = dag;
  }

  /** Get state space diagram phase. */
  public ExecutionPhase getPhase() {
    return diagram.phase;
  }

  /** Upstream getter */
  public List<PartialSchedule> getUpstreams() {
    return upstreams;
  }

  /** Downstream getter */
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

  /** Check if a transition is a default transition. */
  public static boolean isDefaultTransition(List<Instruction> transition) {
    return transition.size() == 1 && (transition.get(0) instanceof JAL);
  }

  /**
   * Connect two fragments with a default transition (no guards). Changing the default transition
   * here would require changing isDefaultTransition() also.
   */
  public static void linkSchedulesWithDefaultTransition(
      PartialSchedule upstream, PartialSchedule downstream) {
    List<Instruction> defaultTransition =
        Arrays.asList(
            new JAL(
                ReturnAddr.ABSTRACT_REGISTER,
                Label.getExecutionPhaseLabel(downstream.getPhase()))); // Default transition
    upstream.addDownstream(downstream, defaultTransition);
    downstream.addUpstream(upstream);
  }

  /** Connect two fragments with a guarded transition. */
  public static void linkSchedulesWithGuardedTransition(
      PartialSchedule upstream, PartialSchedule downstream, List<Instruction> guardedTransition) {
    upstream.addDownstream(downstream, guardedTransition);
    downstream.addUpstream(upstream);
  }

  /**
   * Given a list of partial schedules with non-empty SSDs, link them using the phase information in
   * SSDs.
   */
  public static void link(List<PartialSchedule> schedules, Registers registers) {
    var init = schedules.stream().filter(it -> it.getPhase() == ExecutionPhase.INIT).findFirst();
    var periodic =
        schedules.stream().filter(it -> it.getPhase() == ExecutionPhase.PERIODIC).findFirst();
    var timeout =
        schedules.stream()
            .filter(it -> it.getPhase() == ExecutionPhase.SHUTDOWN_TIMEOUT)
            .findFirst();

    // If INIT and PERIODIC are present, connect them with a default transition.
    if (init.isPresent() && periodic.isPresent()) {
      linkSchedulesWithDefaultTransition(init.get(), periodic.get());
    }

    // If there is TIMEOUT, check if there is periodic, if so, link PERIODIC and
    // TIMEOUT. Otherwise, link INIT and TIMEOUT.
    if (timeout.isPresent()) {
      // Generate a guarded transition.
      // Only transition to this fragment when offset >= timeout.
      List<Instruction> guardedTransition = new ArrayList<>();
      guardedTransition.add(
          new BGE(
              registers.offset,
              registers.timeout,
              Label.getExecutionPhaseLabel(ExecutionPhase.SHUTDOWN_TIMEOUT)));

      // If PERIODIC is present, link PERIODIC to shutdown.
      if (periodic.isPresent()) {
        linkSchedulesWithGuardedTransition(periodic.get(), timeout.get(), guardedTransition);
      }
      // If INIT is present, link INIT to shutdown.
      else if (init.isPresent()) {
        linkSchedulesWithGuardedTransition(init.get(), timeout.get(), guardedTransition);
      }
    } else {
      // If there isn't timeout, link PERIODIC back to itself.
      if (periodic.isPresent()) {
        linkSchedulesWithDefaultTransition(periodic.get(), periodic.get());
      }
    }
  }
}
