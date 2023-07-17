package org.lflang.analyses.dag;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;
import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.analyses.statespace.StateSpaceNode;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CFileConfig;

/**
 * Constructs a Directed Acyclic Graph (Dag) from the State Space Diagram. This is part of the
 * static schedule generation.
 *
 * @author Chadlia Jerad
 * @author Shaokai Lin
 */
public class DagGenerator {

  /** File config */
  public final CFileConfig fileConfig;

  /**
   * Constructor. Sets the main reactor and initializes the dag
   *
   * @param main main reactor instance
   */
  public DagGenerator(CFileConfig fileConfig) {
    this.fileConfig = fileConfig;
  }

  /**
   * The state space diagram, together with the lf program topology and priorities, are used to
   * generate the Dag. Only state space diagrams without loops or without an initialization phase
   * can successfully generate DAGs.
   */
  public Dag generateDag(StateSpaceDiagram stateSpaceDiagram) {
    if (stateSpaceDiagram.isCyclic()) return generateDagForCyclicDiagram(stateSpaceDiagram);
    else return generateDagForAcyclicDiagram(stateSpaceDiagram);
  }

  public Dag generateDagForAcyclicDiagram(StateSpaceDiagram stateSpaceDiagram) {
    // Variables
    Dag dag = new Dag();
    StateSpaceNode currentStateSpaceNode = stateSpaceDiagram.head;
    TimeValue previousTime = TimeValue.ZERO;
    DagNode previousSync = null;
    final TimeValue timeOffset = stateSpaceDiagram.head.time;

    // Check if a DAG can be generated for the given state space diagram.
    // Only a diagram without a loop or a loopy diagram without an
    // initialization phase can generate the DAG.

    ArrayList<DagNode> currentReactionNodes = new ArrayList<>();
    ArrayList<DagNode> reactionsUnconnectedToSync = new ArrayList<>();

    DagNode sync = null; // Local variable for tracking the current SYNC node.
    while (currentStateSpaceNode != null) {

      // Get the current logical time. Or, if this is the last iteration,
      // set the loop period as the logical time.
      TimeValue time = currentStateSpaceNode.time.sub(timeOffset);

      // Add a SYNC node.
      sync = dag.addNode(DagNode.dagNodeType.SYNC, time);
      if (dag.head == null) dag.head = sync;

      // Create DUMMY and Connect SYNC and previous SYNC to DUMMY
      if (!time.equals(TimeValue.ZERO)) {
        TimeValue timeDiff = time.sub(previousTime);
        DagNode dummy = dag.addNode(DagNode.dagNodeType.DUMMY, timeDiff);
        dag.addEdge(previousSync, dummy);
        dag.addEdge(dummy, sync);
      }

      // Add reaction nodes, as well as the edges connecting them to SYNC.
      currentReactionNodes.clear();
      for (ReactionInstance reaction : currentStateSpaceNode.reactionsInvoked) {
        DagNode node = dag.addNode(DagNode.dagNodeType.REACTION, reaction);
        currentReactionNodes.add(node);
        dag.addEdge(sync, node);
      }

      // Now add edges based on reaction dependencies.
      for (DagNode n1 : currentReactionNodes) {
        for (DagNode n2 : currentReactionNodes) {
          if (n1.nodeReaction.dependentReactions().contains(n2.nodeReaction)) {
            dag.addEdge(n1, n2);
          }
        }
      }

      // Collect a set of ReactorInstances from currentReactionNodes.
      Set<ReactorInstance> currentReactors =
          currentReactionNodes.stream()
              .map(DagNode::getReaction)
              .map(ReactionInstance::getParent)
              .collect(Collectors.toCollection(HashSet::new));

      // When generating DAGs, connect an earlier reaction invocation to a SYNC
      // node before releasing a future reaction invocation from the same
      // reactor.
      //
      // FIXME: This assumes that the (conventional) deadline is the
      // period. We need to find a way to integrate LF deadlines into
      // the picture.
      ArrayList<DagNode> toRemove = new ArrayList<>();
      for (DagNode n : reactionsUnconnectedToSync) {
        if (currentReactors.contains(n.nodeReaction.getParent())) {
          dag.addEdge(n, sync);
          toRemove.add(n);
        }
      }
      reactionsUnconnectedToSync.removeAll(toRemove);
      reactionsUnconnectedToSync.addAll(currentReactionNodes);

      // Move to the next state space node.
      currentStateSpaceNode = stateSpaceDiagram.getDownstreamNode(currentStateSpaceNode);
      previousSync = sync;
      previousTime = time;
    }

    // Set the time of the last SYNC node to be the tag of the first pending
    // event in the tail node of the state space diagram.
    // Assumption: this assumes that the heap-to-arraylist convertion puts the
    // earliest event in the first location in arraylist.
    TimeValue time;
    if (stateSpaceDiagram.tail.eventQ.size() > 0)
      time = new TimeValue(stateSpaceDiagram.tail.eventQ.get(0).tag.timestamp, TimeUnit.NANO);
    // If there are no pending events, set the time of the last SYNC node to forever.
    else time = TimeValue.MAX_VALUE;

    // Wrap-up procedure
    wrapup(dag, time, previousSync, previousTime, reactionsUnconnectedToSync);

    return dag;
  }

  public Dag generateDagForCyclicDiagram(StateSpaceDiagram stateSpaceDiagram) {
    // Variables
    Dag dag = new Dag();
    StateSpaceNode currentStateSpaceNode = stateSpaceDiagram.head;
    TimeValue previousTime = TimeValue.ZERO;
    DagNode previousSync = null;
    int counter = 0;
    final TimeValue timeOffset = stateSpaceDiagram.head.time;

    // Check if a DAG can be generated for the given state space diagram.
    // Only a diagram without a loop or a loopy diagram without an
    // initialization phase can generate the DAG.

    ArrayList<DagNode> currentReactionNodes = new ArrayList<>();
    ArrayList<DagNode> reactionsUnconnectedToSync = new ArrayList<>();

    DagNode sync = null; // Local variable for tracking the current SYNC node.
    while (true) {
      // If the current node is the loop node.
      // The stop condition is when the loop node is encountered the 2nd time.
      if (currentStateSpaceNode == stateSpaceDiagram.loopNode) {
        counter++;
        if (counter >= 2) break;
      }

      // Get the current logical time. Or, if this is the last iteration,
      // set the loop period as the logical time.
      TimeValue time = currentStateSpaceNode.time.sub(timeOffset);

      // Add a SYNC node.
      sync = dag.addNode(DagNode.dagNodeType.SYNC, time);
      if (dag.head == null) dag.head = sync;

      // Create DUMMY and Connect SYNC and previous SYNC to DUMMY
      if (!time.equals(TimeValue.ZERO)) {
        TimeValue timeDiff = time.sub(previousTime);
        DagNode dummy = dag.addNode(DagNode.dagNodeType.DUMMY, timeDiff);
        dag.addEdge(previousSync, dummy);
        dag.addEdge(dummy, sync);
      }

      // Add reaction nodes, as well as the edges connecting them to SYNC.
      currentReactionNodes.clear();
      for (ReactionInstance reaction : currentStateSpaceNode.reactionsInvoked) {
        DagNode node = dag.addNode(DagNode.dagNodeType.REACTION, reaction);
        currentReactionNodes.add(node);
        dag.addEdge(sync, node);
      }

      // Now add edges based on reaction dependencies.
      for (DagNode n1 : currentReactionNodes) {
        for (DagNode n2 : currentReactionNodes) {
          if (n1.nodeReaction.dependentReactions().contains(n2.nodeReaction)) {
            dag.addEdge(n1, n2);
          }
        }
      }

      // Collect a set of ReactorInstances from currentReactionNodes.
      Set<ReactorInstance> currentReactors =
          currentReactionNodes.stream()
              .map(DagNode::getReaction)
              .map(ReactionInstance::getParent)
              .collect(Collectors.toCollection(HashSet::new));

      // When generating DAGs, connect an earlier reaction invocation to a SYNC
      // node before releasing a future reaction invocation from the same
      // reactor.
      //
      // FIXME: This assumes that the (conventional) deadline is the
      // period. We need to find a way to integrate LF deadlines into
      // the picture.
      ArrayList<DagNode> toRemove = new ArrayList<>();
      for (DagNode n : reactionsUnconnectedToSync) {
        if (currentReactors.contains(n.nodeReaction.getParent())) {
          dag.addEdge(n, sync);
          toRemove.add(n);
        }
      }
      reactionsUnconnectedToSync.removeAll(toRemove);
      reactionsUnconnectedToSync.addAll(currentReactionNodes);

      // Move to the next state space node.
      currentStateSpaceNode = stateSpaceDiagram.getDownstreamNode(currentStateSpaceNode);
      previousSync = sync;
      previousTime = time;
    }

    // Set the time of the last SYNC node to be the hyperperiod.
    TimeValue time = new TimeValue(stateSpaceDiagram.hyperperiod, TimeUnit.NANO);

    // Wrap-up procedure
    wrapup(dag, time, previousSync, previousTime, reactionsUnconnectedToSync);

    return dag;
  }

  /** A wrap-up procedure */
  private void wrapup(
      Dag dag,
      TimeValue time,
      DagNode previousSync,
      TimeValue previousTime,
      ArrayList<DagNode> reactionsUnconnectedToSync) {
    // Add a SYNC node.
    DagNode sync = dag.addNode(DagNode.dagNodeType.SYNC, time);
    if (dag.head == null) dag.head = sync;

    // Create DUMMY and Connect SYNC and previous SYNC to DUMMY
    if (!time.equals(TimeValue.ZERO)) {
      TimeValue timeDiff = time.sub(previousTime);
      DagNode dummy = dag.addNode(DagNode.dagNodeType.DUMMY, timeDiff);
      dag.addEdge(previousSync, dummy);
      dag.addEdge(dummy, sync);
    }

    // Add edges from existing reactions to the last node,
    // and break the loop before adding more reaction nodes.
    for (DagNode n : reactionsUnconnectedToSync) {
      dag.addEdge(n, sync);
    }

    // After exiting the while loop, assign the last SYNC node as tail.
    dag.tail = sync;
  }
}
