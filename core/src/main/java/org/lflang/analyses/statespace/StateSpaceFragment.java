package org.lflang.analyses.statespace;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.lflang.pretvm.ExecutionPhase;
import org.lflang.pretvm.instruction.Instruction;

/**
 * A state space fragment contains a state space diagram and references to other state space
 * diagrams. A fragment is meant to capture partial behavior of an LF program (for example, the
 * initialization phase, periodic phase, or shutdown phases).
 */
public class StateSpaceFragment {

  /**
   * A static fragment for the EPILOGUE phase. Static fragments do not go into the fragments list
   * and their instructions are directly injected at link time. The EPILOGUE static fragment is only
   * here to make sure fragments generated in generateStateSpaceFragments() properly transition to
   * the EPILOGUE after they are done. There is no need to have another static PREAMBLE fragment,
   * since no fragments transition into PREAMBLE.
   */
  public static final StateSpaceFragment EPILOGUE;

  static {
    StateSpaceDiagram epilogueDiagram = new StateSpaceDiagram();
    epilogueDiagram.phase = ExecutionPhase.EPILOGUE;
    EPILOGUE = new StateSpaceFragment(epilogueDiagram);
  }

  /** The state space diagram contained in this fragment */
  StateSpaceDiagram diagram;

  /** A list of upstream fragments */
  List<StateSpaceFragment> upstreams = new ArrayList<>();

  /**
   * A map from downstream fragments to their guard. A guard is a list of instructions representing
   * a conditional branch. A null value indicates a default (unconditional) transition.
   */
  Map<StateSpaceFragment, List<Instruction<?, ?, ?>>> downstreams = new HashMap<>();

  /** Constructor */
  public StateSpaceFragment() {}

  /** Constructor */
  public StateSpaceFragment(StateSpaceDiagram diagram) {
    this.diagram = diagram;
  }

  /** Check if the fragment is cyclic. */
  public boolean isCyclic() {
    return diagram.isCyclic();
  }

  /** Diagram getter */
  public StateSpaceDiagram getDiagram() {
    return diagram;
  }

  /** Get state space diagram phase. */
  public ExecutionPhase getPhase() {
    return diagram.phase;
  }

  /** Upstream getter */
  public List<StateSpaceFragment> getUpstreams() {
    return upstreams;
  }

  /** Downstream getter */
  public Map<StateSpaceFragment, List<Instruction<?, ?, ?>>> getDownstreams() {
    return downstreams;
  }

  /** Add an upstream fragment */
  public void addUpstream(StateSpaceFragment upstream) {
    this.upstreams.add(upstream);
  }

  /** Add a downstream fragment with a guarded transition */
  public void addDownstream(StateSpaceFragment downstream, List<Instruction<?, ?, ?>> guard) {
    this.downstreams.put(downstream, guard);
  }
}
