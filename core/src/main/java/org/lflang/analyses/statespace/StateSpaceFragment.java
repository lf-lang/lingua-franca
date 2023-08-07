package org.lflang.analyses.statespace;

import java.util.ArrayList;
import java.util.List;

/**
 * A state space fragment contains a state space diagram and references to other state space
 * diagrams. A fragment is meant to capture partial behavior of an LF program (for example, the
 * initialization phase, periodic phase, or shutdown phase).
 *
 * <p>FIXME: Add predicates to transitions between fragments.
 *
 * @author Shaokai Lin
 */
public class StateSpaceFragment {

  /** The state space diagram contained in this fragment */
  StateSpaceDiagram diagram;

  /** Point to an upstream fragment */
  List<StateSpaceFragment> upstreams = new ArrayList<>();

  /** Point to a downstream fragment */
  List<StateSpaceFragment> downstreams = new ArrayList<>();

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

  /** Upstream getter */
  public List<StateSpaceFragment> getUpstreams() {
    return upstreams;
  }

  /** Downstream getter */
  public List<StateSpaceFragment> getDownstreams() {
    return downstreams;
  }

  /** Upstream setter */
  public void addUpstream(StateSpaceFragment upstream) {
    this.upstreams.add(upstream);
  }

  /** Downstream setter */
  public void addDownstream(StateSpaceFragment downstream) {
    this.downstreams.add(downstream);
  }
}
