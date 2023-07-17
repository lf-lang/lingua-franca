package org.lflang.analyses.statespace;

/** A fragment is a part of a state space diagram */
public class StateSpaceFragment extends StateSpaceDiagram {

  /** Point to an upstream fragment */
  StateSpaceFragment upstream;

  /** Point to a downstream fragment */
  StateSpaceFragment downstream;

  /** Upstream getter */
  public StateSpaceFragment getUpstream() {
    return upstream;
  }

  /** Downstream getter */
  public StateSpaceFragment getDownstream() {
    return downstream;
  }
}
