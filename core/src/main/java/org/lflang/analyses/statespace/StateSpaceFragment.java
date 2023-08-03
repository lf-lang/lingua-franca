package org.lflang.analyses.statespace;

/**
 * A state space fragment contains a state space diagram and references to other state space
 * diagrams. A fragment is meant to capture partial behavior of an LF program (for example, the
 * initialization phase, periodic phase, or shutdown phase).
 *
 * <p>FIXME: Turn upstream and downstream into lists, and add predicates to transitions between
 * fragments.
 *
 * @author Shaokai Lin
 */
public class StateSpaceFragment {

  /** The state space diagram contained in this fragment */
  StateSpaceDiagram diagram;

  /** Point to an upstream fragment */
  StateSpaceFragment upstream;

  /** Point to a downstream fragment */
  StateSpaceFragment downstream;

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
  public StateSpaceFragment getUpstream() {
    return upstream;
  }

  /** Downstream getter */
  public StateSpaceFragment getDownstream() {
    return downstream;
  }

  /** Upstream setter */
  public void setUpstream(StateSpaceFragment upstream) {
    this.upstream = upstream;
  }

  /** Downstream setter */
  public void setDownstream(StateSpaceFragment downstream) {
    this.downstream = downstream;
  }
}
