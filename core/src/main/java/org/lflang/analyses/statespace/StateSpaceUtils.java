package org.lflang.analyses.statespace;

import java.nio.file.Path;
import java.util.ArrayList;

public class StateSpaceUtils {

  /**
   * Identify an initialization phase and a periodic phase of the state space diagram, and create
   * two different state space fragments.
   */
  public static ArrayList<StateSpaceFragment> fragmentizeForDagGen(
      StateSpaceDiagram stateSpace, Path dotFileDir) {

    stateSpace.display();

    ArrayList<StateSpaceFragment> fragments = new ArrayList<>();
    StateSpaceNode current = stateSpace.head;
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
      if (stateSpace.loopNode != null)
        initPhase.hyperperiod = stateSpace.loopNode.getTime().toNanoSeconds();
      else initPhase.hyperperiod = 0;
      fragments.add(initPhase);
    }

    // Create a periodic phase fragment.
    if (stateSpace.isCyclic()) {

      // State this assumption explicitly.
      assert current == stateSpace.loopNode : "Current is not pointing to loopNode.";

      StateSpaceFragment periodicPhase = new StateSpaceFragment();
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
      fragments.add(periodicPhase);
    }

    // Make fragments refer to each other.
    if (fragments.size() == 2) {
      fragments.get(0).downstream = fragments.get(1);
      fragments.get(1).upstream = fragments.get(0);
    }

    // Pretty print for debugging
    System.out.println(fragments.size() + " fragments added.");
    for (int i = 0; i < fragments.size(); i++) {
      var f = fragments.get(i);
      f.display();

      // Generate a dot file.
      Path file = dotFileDir.resolve("state_space_frag_" + i + ".dot");
      f.generateDotFile(file);
    }

    assert fragments.size() <= 2 : "More than two fragments detected!";
    return fragments;
  }
}
