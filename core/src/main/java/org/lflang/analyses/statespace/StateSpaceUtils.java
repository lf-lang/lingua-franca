package org.lflang.analyses.statespace;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import org.lflang.TimeTag;
import org.lflang.TimeValue;
import org.lflang.generator.ReactorInstance;
import org.lflang.pretvm.ExecutionPhase;
import org.lflang.pretvm.instruction.Instruction;
import org.lflang.target.TargetConfig;

/** A utility class for state space-related methods. */
public class StateSpaceUtils {

  /**
   * Connect two fragments with a default (unconditional) transition. A null guard indicates a
   * default transition. The actual transition instruction (e.g., JAL) is generated during code
   * generation when concrete register instances are available.
   */
  public static void connectFragmentsDefault(
      StateSpaceFragment upstream, StateSpaceFragment downstream) {
    upstream.addDownstream(downstream, null);
    downstream.addUpstream(upstream);
  }

  /** Connect two fragments with a guarded transition. */
  public static void connectFragmentsGuarded(
      StateSpaceFragment upstream,
      StateSpaceFragment downstream,
      List<Instruction<?, ?, ?>> guardedTransition) {
    upstream.addDownstream(downstream, guardedTransition);
    downstream.addUpstream(upstream);
  }

  /**
   * A helper function that generates a state space diagram for an LF program based on an
   * exploration phase.
   */
  public static StateSpaceDiagram generateStateSpaceDiagram(
      ExecutionPhase explorePhase,
      ReactorInstance main,
      TimeTag horizon,
      TargetConfig targetConfig,
      Path graphDir,
      String graphPrefix) {
    StateSpaceDiagram stateSpaceDiagram =
        StateSpaceExplorer.explore(main, horizon, explorePhase, targetConfig);
    if (!stateSpaceDiagram.isEmpty()) {
      stateSpaceDiagram.generateDotFile(graphDir, graphPrefix + ".dot");
    }
    return stateSpaceDiagram;
  }

  /**
   * A helper function that generates state space diagrams for asynchronous events (physical
   * actions). Each physical action gets its own diagram.
   */
  public static List<StateSpaceDiagram> generateAsyncStateSpaceDiagrams(
      ExecutionPhase explorePhase,
      ReactorInstance main,
      TargetConfig targetConfig,
      Path graphDir,
      String graphPrefix) {
    if (explorePhase != ExecutionPhase.ASYNC)
      throw new RuntimeException(
          "The exploration mode must be ASYNC inside generateAsyncStateSpaceDiagrams().");

    List<StateSpaceDiagram> diagramList = new ArrayList<>();

    // Collect a list of asynchronous events (physical actions).
    List<Event> asyncEvents =
        StateSpaceExplorer.addInitialEvents(main, ExecutionPhase.ASYNC, targetConfig);

    // For each asynchronous event, run the explore function.
    for (int i = 0; i < asyncEvents.size(); i++) {
      Event event = asyncEvents.get(i);

      // Explore the state space with a single async event.
      StateSpaceDiagram stateSpaceDiagram =
          StateSpaceExplorer.explore(main, TimeTag.FOREVER, explorePhase, Arrays.asList(event));

      if (!stateSpaceDiagram.isEmpty()) {
        stateSpaceDiagram.generateDotFile(graphDir, graphPrefix + "_" + i + ".dot");
      }
      diagramList.add(stateSpaceDiagram);
    }

    return diagramList;
  }

  /** Check if a transition is a default (unconditional) transition. */
  public static boolean isDefaultTransition(List<Instruction<?, ?, ?>> transition) {
    return transition == null;
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
   * Merge an async diagram into a non-async diagram based on minimum spacing, which in this case is
   * interpreted as the period at which the presence of the physical action is polled.
   *
   * <p>ASSUMPTIONS:
   *
   * <p>1. min spacing <= hyperperiod. If minimum spacing > hyperperiod, we then need to unroll the
   * synchronous diagram.
   *
   * <p>2. min spacing is a divisor of the hyperperiod. This simplifies the placement of async nodes
   * and removes the need to account for the shift of async node sequence over multiple iterations.
   *
   * <p>3. Physical action does not occur at the same tag as synchronous nodes. This removes the
   * need to merge two nodes at the same tag.
   *
   * <p>4. The sequence of async nodes is shifted 1 nsec after the sync sequence starts. This is a
   * best effort approach to avoid collision between sync and async nodes.
   *
   * <p>5. Sometimes, the actual minimum spacing in the schedule could be greater than the specified
   * minimum spacing. This could occur due to a few reasons: A) Assumption 3; B) In the transition
   * between the initialization phase and the periodic phase, the LAST async node in the
   * initialization phase can be dropped to make sure the FIRST async node in the periodic phase
   * does not break the min spacing requirement.
   */
  public static StateSpaceDiagram mergeAsyncDiagramIntoDiagram(
      StateSpaceDiagram asyncDiagram, StateSpaceDiagram targetDiagram) {

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
    TimeTag asyncTag =
        new TimeTag(
            TimeValue.fromNanoSeconds(targetDiagram.head.getTag().time.toNanoSeconds() + 1), 0L);

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
            new TimeTag(
                TimeValue.fromNanoSeconds(
                    asyncTag.time.toNanoSeconds()
                        + asyncDiagram.getMinSpacing().toNanoSeconds()),
                0L);
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
      }

      if (mergedDiagram.head == null) mergedDiagram.head = lastAdded;

      if (lastAdded == targetDiagram.tail) {
        // Create a new async node.
        StateSpaceNode asyncNodeNew = new StateSpaceNode(asyncNode);
        asyncNodeNew.setTag(asyncTag);
        mergedDiagram.addNode(asyncNodeNew);
        mergedDiagram.addEdge(asyncNodeNew, lastAdded);
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
          mergedDiagram.addEdge(mergedDiagram.loopNode, lastAdded);
        }
        stop = true;
      }
    }

    return mergedDiagram;
  }
}
