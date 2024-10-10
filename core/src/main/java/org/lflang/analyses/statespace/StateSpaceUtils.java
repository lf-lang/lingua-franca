package org.lflang.analyses.statespace;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.lflang.analyses.pretvm.Instruction;
import org.lflang.analyses.pretvm.InstructionJAL;
import org.lflang.analyses.pretvm.Register;
import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;
import org.lflang.generator.ReactorInstance;
import org.lflang.target.TargetConfig;

/**
 * A utility class for state space-related methods
 *
 * @author Shaokai Lin
 */
public class StateSpaceUtils {

  /**
   * Connect two fragments with a default transition (no guards). Changing the default transition
   * here would require changing isDefaultTransition() also.
   */
  public static void connectFragmentsDefault(
      StateSpaceFragment upstream, StateSpaceFragment downstream) {
    List<Instruction> defaultTransition =
        Arrays.asList(
            new InstructionJAL(
                Register.ABSTRACT_WORKER_RETURN_ADDR, downstream.getPhase())); // Default transition
    upstream.addDownstream(downstream, defaultTransition);
    downstream.addUpstream(upstream);
  }

  /** Connect two fragments with a guarded transition. */
  public static void connectFragmentsGuarded(
      StateSpaceFragment upstream,
      StateSpaceFragment downstream,
      List<Instruction> guardedTransition) {
    upstream.addDownstream(downstream, guardedTransition);
    downstream.addUpstream(upstream);
  }

  /**
   * A helper function that generates a state space diagram for an LF program based on an
   * exploration phase.
   */
  public static StateSpaceDiagram generateStateSpaceDiagram(
      StateSpaceExplorer explorer,
      StateSpaceExplorer.Phase explorePhase,
      ReactorInstance main,
      Tag horizon,
      TargetConfig targetConfig,
      Path graphDir,
      String graphPrefix) {
    // Get a list of initial events according to the exploration phase.
    List<Event> initialEvents =
        StateSpaceExplorer.addInitialEvents(main, explorePhase, targetConfig);

    // Explore the state space with the phase specified.
    StateSpaceDiagram stateSpaceDiagram =
        explorer.explore(main, horizon, explorePhase, initialEvents);

    // Generate a dot file.
    if (!stateSpaceDiagram.isEmpty()) {
      Path file = graphDir.resolve(graphPrefix + ".dot");
      stateSpaceDiagram.generateDotFile(file);
    }

    return stateSpaceDiagram;
  }

  /**
   * A helper function that generates a state space diagram for an LF program based on an
   * exploration phase.
   */
  public static List<StateSpaceDiagram> generateAsyncStateSpaceDiagrams(
      StateSpaceExplorer explorer,
      StateSpaceExplorer.Phase explorePhase,
      ReactorInstance main,
      Tag horizon,
      TargetConfig targetConfig,
      Path graphDir,
      String graphPrefix) {

    // Check if the mode is ASYNC.
    if (explorePhase != Phase.ASYNC)
      throw new RuntimeException(
          "The exploration mode must be ASYNC inside generateStateSpaceDiagramAsync().");

    // Create a list.
    List<StateSpaceDiagram> diagramList = new ArrayList<>();

    // Collect a list of asynchronous events (physical action).
    List<Event> asyncEvents = StateSpaceExplorer.addInitialEvents(main, Phase.ASYNC, targetConfig);

    // For each asynchronous event, run the explore function.
    for (int i = 0; i < asyncEvents.size(); i++) {
      // Get an event.
      Event event = asyncEvents.get(i);

      // Explore the state space with the phase specified.
      StateSpaceDiagram stateSpaceDiagram =
          explorer.explore(main, new Tag(0, 0, true), explorePhase, Arrays.asList(event));

      // Generate a dot file.
      if (!stateSpaceDiagram.isEmpty()) {
        Path file = graphDir.resolve(graphPrefix + "_" + i + ".dot");
        stateSpaceDiagram.generateDotFile(file);
      }

      diagramList.add(stateSpaceDiagram);
    }

    return diagramList;
  }

  /**
   * Identify an initialization phase and a periodic phase of the state space diagram, and create
   * two different state space diagrams.
   */
  public static ArrayList<StateSpaceDiagram> splitInitAndPeriodicDiagrams(
      StateSpaceDiagram stateSpace) {

    ArrayList<StateSpaceDiagram> diagrams = new ArrayList<>();
    StateSpaceNode current = stateSpace.head;
    StateSpaceNode previous = null;

    // Create an initialization phase diagram.
    if (stateSpace.head != stateSpace.loopNode) {
      StateSpaceDiagram initPhase = new StateSpaceDiagram();
      initPhase.head = current;
      while (current != stateSpace.loopNode) {
        // Add node and edges to diagram.
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
      diagrams.add(initPhase);
    }

    // Create a periodic phase diagram.
    if (stateSpace.isCyclic()) {

      // State this assumption explicitly.
      assert current == stateSpace.loopNode : "Current is not pointing to loopNode.";

      StateSpaceDiagram periodicPhase = new StateSpaceDiagram();
      periodicPhase.head = current;
      periodicPhase.addNode(current); // Add the first node.
      if (current == stateSpace.tail) {
        periodicPhase.addEdge(current, current); // Add edges to diagram.
      }
      while (current != stateSpace.tail) {
        // Update current and previous pointer.
        // We bring the updates before addNode() because
        // we need to make sure tail is added.
        // For the init diagram, we do not want to add loopNode.
        previous = current;
        current = stateSpace.getDownstreamNode(current);

        // Add node and edges to diagram.
        periodicPhase.addNode(current);
        periodicPhase.addEdge(current, previous);
      }
      periodicPhase.tail = current;
      periodicPhase.loopNode = stateSpace.loopNode;
      periodicPhase.addEdge(periodicPhase.loopNode, periodicPhase.tail); // Add loop.
      periodicPhase.loopNodeNext = stateSpace.loopNodeNext;
      periodicPhase.hyperperiod = stateSpace.hyperperiod;
      periodicPhase.phase = Phase.PERIODIC;
      diagrams.add(periodicPhase);
    }

    return diagrams;
  }

  /** Check if a transition is a default transition. */
  public static boolean isDefaultTransition(List<Instruction> transition) {
    return transition.size() == 1 && (transition.get(0) instanceof InstructionJAL);
  }

  /**
   * Merge a list of async diagrams into a non-async diagram.
   *
   * <p>FIXME: This is not an efficient algorithm since every time a new diagram gets merged, the
   * size of the merged diagram could blow up exponentially. A one-pass algorithm would be better.
   *
   * @param asyncDiagrams A list of async diagrams to be merged in
   * @param targetDiagram The target diagram accepting async diagrams
   * @return A merged diagram
   */
  public static StateSpaceDiagram mergeAsyncDiagramsIntoDiagram(
      List<StateSpaceDiagram> asyncDiagrams, StateSpaceDiagram targetDiagram) {
    StateSpaceDiagram mergedDiagram = targetDiagram;
    for (var diagram : asyncDiagrams) {
      mergedDiagram = mergeAsyncDiagramIntoDiagram(diagram, targetDiagram);
    }
    return mergedDiagram;
  }

  /**
   * Merge an async diagram into a non-async fragment based on minimum spacing, which in this case
   * is interpreted as the period at which the presence of the physical action.
   *
   * <p>In the state space exploration, generate a diagram for EACH physical action's data path.
   * Then associate a minimum spacing for each diagram. When calling this merge function, the
   * algorithm performs merging based on the indidual minimum spacing specifications.
   *
   * <p>ASSUMPTIONS:
   *
   * <p>1. min spacing <= hyperperiod. If minimum space > hyperperiod, we then need to unroll the
   * synchronous diagram.
   *
   * <p>2. min spacing is a divisor of the hyperperiod. This simplifies the placement of async nodes
   * and removes the need to account for the shift of async node sequence over multiple iterations.
   * To relax this assumption, we need to recalculate the hyperperiod of a periodic diagram with the
   * addition of async nodes.
   *
   * <p>3. Physical action does not occur at the same tag as synchronous nodes. This removes the
   * need to merge two nodes at the same tag. This is not necessarily hard, however.
   *
   * <p>4. The sequence of async nodes is shifted 1 nsec after the sync sequence starts. This is a
   * best effort approach to avoid collision between sync and async nodes, in which case assumption
   * 3 is activated. This also makes it easy for the merge algorithm to work on a single diagram
   * without worrying about the temporal relations between its async nodes and other diagrams' async
   * nodes, i.e., enabling a compositional merge strategy.
   *
   * <p>5. Sometimes, the actual minimum spacing in the schedule could be greater than the specified
   * minimum spacing. This could occur due to a few reasons: A) Assumption 3; B) In the transition
   * between the initialization phase and the periodic phase, the LAST async node in the
   * initialization phase can be dropped to make sure the FIRST async node in the periodic phase
   * does not break the min spacing requirement due to compositional merge strategy. This is not a
   * problem if we consider a global merge strategy, i.e., using a single diagram to represent the
   * logical behavior and merge the async nodes across multiple phases at once.
   */
  public static StateSpaceDiagram mergeAsyncDiagramIntoDiagram(
      StateSpaceDiagram asyncDiagram, StateSpaceDiagram targetDiagram) {
    System.out.println("*** Inside merge algorithm.");

    StateSpaceDiagram mergedDiagram = new StateSpaceDiagram();

    // Inherit phase from targetDiagram
    mergedDiagram.phase = targetDiagram.phase;

    // Keep track of the current node of the target diagram.
    StateSpaceNode current = targetDiagram.head;

    // Tracking the lastAdded of the merged diagram.
    StateSpaceNode lastAdded = null;

    // Assuming the async diagram only has 1 node.
    StateSpaceNode asyncNode = asyncDiagram.head;

    // Set the first tag of the async node to be 1 nsec after the head of the
    // target diagram. This is a best effort approach to avoid tag collision
    // between the synchronous sequence and the asynchronous sequence.
    Tag asyncTag = new Tag(targetDiagram.head.getTag().timestamp + 1, 0, false);

    System.out.println("asyncTag = " + asyncTag);

    boolean stop = false;
    while (!stop) {

      // Decide if async node or the current node should be added.
      if (asyncTag.compareTo(current.getTag()) < 0) {
        // Create a new async node.
        StateSpaceNode asyncNodeNew = new StateSpaceNode(asyncNode);
        asyncNodeNew.setTag(asyncTag);
        mergedDiagram.addNode(asyncNodeNew);
        mergedDiagram.addEdge(asyncNodeNew, lastAdded);

        // Update lastAdded
        lastAdded = asyncNodeNew;

        // Update async tag
        asyncTag =
            new Tag(asyncTag.timestamp + asyncDiagram.getMinSpacing().toNanoSeconds(), 0, false);

        System.out.println("Added async node.");
      } else {

        // Add the current node of the synchronous diagram.
        mergedDiagram.addNode(current);
        mergedDiagram.addEdge(current, lastAdded);

        // Check if the current node is the loop node.
        // If so, set it to the loop node in the new diagram.
        if (current == targetDiagram.loopNode) mergedDiagram.loopNode = current;

        // Update current and previous pointer.
        lastAdded = current;
        current = targetDiagram.getDownstreamNode(current);

        System.out.println("Added current node.");
      }

      if (mergedDiagram.head == null) mergedDiagram.head = lastAdded;

      if (lastAdded == targetDiagram.tail) {
        // Create a new async node.
        StateSpaceNode asyncNodeNew = new StateSpaceNode(asyncNode);
        asyncNodeNew.setTag(asyncTag);
        mergedDiagram.addNode(asyncNodeNew);
        mergedDiagram.addEdge(asyncNodeNew, lastAdded);
        System.out.println("Added async node.");
        // Update lastAdded
        lastAdded = asyncNodeNew;
        // Set tail
        mergedDiagram.tail = lastAdded;
        // Inherit loopNodeNext from targetDiagram
        mergedDiagram.loopNodeNext = targetDiagram.loopNodeNext;
        // Inherit hyperperiod.
        mergedDiagram.hyperperiod = targetDiagram.hyperperiod;
        // Connect back to the loop node, if any.
        if (targetDiagram.loopNode != null) {
          System.out.println("targetDiagram.loopNode != null");
          targetDiagram.loopNode.display();
          mergedDiagram.addEdge(mergedDiagram.loopNode, lastAdded);
        } else {
          System.out.println("targetDiagram.loopNode == null!");
        }
        stop = true;
      }
    }

    // FIXME: Display merged diagram
    mergedDiagram.display();

    return mergedDiagram;
  }
}
