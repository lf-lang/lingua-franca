package org.lflang.analyses.statespace;

import java.util.ArrayList;
import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;

/**
 * A utility class for state space-related methods
 *
 * @author Shaokai Lin
 */
public class StateSpaceUtils {

  /**
   * Identify an initialization phase and a periodic phase of the state space diagram, and create
   * two different state space fragments.
   */
  public static ArrayList<StateSpaceFragment> fragmentizeInitAndPeriodic(
      StateSpaceDiagram stateSpace) {

    ArrayList<StateSpaceFragment> fragments = new ArrayList<>();
    StateSpaceNode current = stateSpace.head;
    StateSpaceNode previous = null;

    // Create an initialization phase fragment.
    if (stateSpace.head != stateSpace.loopNode) {
      StateSpaceDiagram initPhase = new StateSpaceDiagram();
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
      if (stateSpace.loopNode != null)
        initPhase.hyperperiod = stateSpace.loopNode.getTime().toNanoSeconds();
      else initPhase.hyperperiod = 0;
      initPhase.phase = Phase.INIT;
      fragments.add(new StateSpaceFragment(initPhase));
    }

    // Create a periodic phase fragment.
    if (stateSpace.isCyclic()) {

      // State this assumption explicitly.
      assert current == stateSpace.loopNode : "Current is not pointing to loopNode.";

      StateSpaceDiagram periodicPhase = new StateSpaceDiagram();
      periodicPhase.head = current;
      periodicPhase.addNode(current); // Add the first node.
      if (current == stateSpace.tail) {
        periodicPhase.addEdge(current, current); // Add edges to fragment.
      }
      while (current != stateSpace.tail) {
        // Update current and previous pointer.
        // We bring the updates before addNode() because
        // we need to make sure tail is added.
        // For the init. fragment, we do not want to add loopNode.
        previous = current;
        current = stateSpace.getDownstreamNode(current);

        // Add node and edges to fragment.
        periodicPhase.addNode(current);
        periodicPhase.addEdge(current, previous);
      }
      periodicPhase.tail = current;
      periodicPhase.loopNode = stateSpace.loopNode;
      periodicPhase.addEdge(periodicPhase.loopNode, periodicPhase.tail); // Add loop.
      periodicPhase.loopNodeNext = stateSpace.loopNodeNext;
      periodicPhase.hyperperiod = stateSpace.hyperperiod;
      periodicPhase.phase = Phase.PERIODIC;
      fragments.add(new StateSpaceFragment(periodicPhase));
    }

    // If there are exactly two fragments (init and periodic),
    // make fragments refer to each other.
    if (fragments.size() == 2) connectFragments(fragments.get(0), fragments.get(1));

    assert fragments.size() <= 2 : "More than two fragments detected!";
    return fragments;
  }

  /**
   * Connect two fragments by calling setDownstream() and setUpstream() on two fragments separately.
   */
  public static void connectFragments(StateSpaceFragment upstream, StateSpaceFragment downstream) {
    upstream.addDownstream(downstream);
    downstream.addUpstream(upstream);
  }
}
