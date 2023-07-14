package org.lflang.analyses.statespace;

import java.util.ArrayList;

public class StateSpaceUtils {

  /** Identify an initialization phase and a periodic phase of the state space
   * diagram, and create two different state space fragments. */
  public static ArrayList<StateSpaceFragment> fragmentizeForDagGen(
      StateSpaceDiagram stateSpace) {
    
    stateSpace.display();

    ArrayList<StateSpaceFragment> fragments = new ArrayList<>();
    StateSpaceNode current  = stateSpace.head;
    StateSpaceNode previous = null;

    // Create an initialization phase fragment.
    if (stateSpace.head != stateSpace.loopNode) {
      StateSpaceFragment initPhase = new StateSpaceFragment();
      initPhase.head = current;
      while (current != stateSpace.loopNode) {
        // Add node and edges to fragment.
        initPhase.addNode(current);
        initPhase.addEdge(current, previous);

        // Update current and previous pointer.
        previous = current;
        current = stateSpace.getDownstreamNode(current);
      }
      initPhase.tail = previous;
      fragments.add(initPhase);
    }

    // Create a periodic phase fragment.
    if (stateSpace.loopNode != null) {
      StateSpaceFragment periodicPhase = new StateSpaceFragment();
      periodicPhase.head = current;
      while (current != stateSpace.tail) {
        // Add node and edges to fragment.
        periodicPhase.addNode(current);
        periodicPhase.addEdge(current, previous);

        // Update current and previous pointer.
        previous = current;
        current = stateSpace.getDownstreamNode(current);
      }
      periodicPhase.tail = current;
      periodicPhase.loopNode = stateSpace.loopNode;
      periodicPhase.loopNodeNext = stateSpace.loopNodeNext;
      fragments.add(periodicPhase);
    }

    // Make fragments refer to each other.
    if (fragments.size() == 2) {
      fragments.get(0).downstream = fragments.get(1);
      fragments.get(1).upstream = fragments.get(0);
    }

    // Pretty print for debugging
    for (var f : fragments) {
      f.display();
    }

    assert fragments.size() <= 2 : "More than two fragments detected!";
    return fragments;
  }
}
