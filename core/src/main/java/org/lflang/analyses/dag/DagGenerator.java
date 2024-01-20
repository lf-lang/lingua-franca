package org.lflang.analyses.dag;

import java.util.ArrayList;
import java.util.stream.Collectors;
import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;
import org.lflang.analyses.statespace.StateSpaceNode;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CFileConfig;

/**
 * Constructs a Directed Acyclic Graph (DAG) from the State Space Diagram. This is part of the
 * static schedule generation.
 *
 * <p>FIXME: DAG generation does not need to be stateful. The methods in this class can be
 * refactored into static methods.
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
    // Variables
    Dag dag = new Dag();
    StateSpaceNode currentStateSpaceNode = stateSpaceDiagram.head;
    TimeValue previousTime = TimeValue.ZERO;
    DagNode previousSync = null;
    final TimeValue timeOffset = stateSpaceDiagram.head.getTime();
    int loopNodeCounter = 0; // Only used when the diagram is cyclic.
    ArrayList<DagNode> currentReactionNodes = new ArrayList<>();
    ArrayList<DagNode> reactionsUnconnectedToSync = new ArrayList<>();
    ArrayList<DagNode> reactionsUnconnectedToNextInvocation = new ArrayList<>();
    DagNode sync = null; // Local variable for tracking the current SYNC node.

    // Check if a DAG can be generated for the given state space diagram.
    // Only a diagram without a loop or a loopy diagram without an
    // initialization phase can generate the DAG.

    while (true) {
      // Check stop conditions based on whether the diagram is cyclic or not.
      if (stateSpaceDiagram.isCyclic()) {
        // If the current node is the loop node.
        // The stop condition is when the loop node is encountered the 2nd time.
        if (currentStateSpaceNode == stateSpaceDiagram.loopNode) {
          loopNodeCounter++;
          if (loopNodeCounter >= 2) break;
        }
      } else {
        if (currentStateSpaceNode == null) break;
      }

      // Get the current logical time. Or, if this is the last iteration,
      // set the loop period as the logical time.
      TimeValue time = currentStateSpaceNode.getTime().sub(timeOffset);

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
      for (ReactionInstance reaction : currentStateSpaceNode.getReactionsInvoked()) {
        DagNode node = dag.addNode(DagNode.dagNodeType.REACTION, reaction);
        currentReactionNodes.add(node);
        dag.addEdge(sync, node);
        node.setAssociatedSyncNode(sync);
      }

      // Now add edges based on reaction dependencies and priorities.
      for (DagNode n1 : currentReactionNodes) {
        for (DagNode n2 : currentReactionNodes) {
          // Add an edge for the set of immediate downstream reactions, which
          // are reactions that receive data produced by this reaction plus at
          // most ONE reaction in the same reactor whose definition lexically
          // follows this one.
          if (n1.nodeReaction.dependentReactions().contains(n2.nodeReaction)) {
            dag.addEdge(n1, n2);
          }
          // Add an edge for reactions in the same reactor based on priorities.
          // This adds the remaining dependencies not accounted for in
          // dependentReactions(), e.g., reaction 3 depends on reaction 1 in the
          // same reactor.
          if (n1.nodeReaction.getParent() == n2.nodeReaction.getParent()
            && n1.nodeReaction.index < n2.nodeReaction.index) {
            dag.addEdge(n1, n2);
          }
        }
      }

      // Create a list of ReactionInstances from currentReactionNodes.
      ArrayList<ReactionInstance> currentReactions =
          currentReactionNodes.stream()
              .map(DagNode::getReaction)
              .collect(Collectors.toCollection(ArrayList::new));

      // If there is a newly released reaction found and its prior
      // invocation is not connected to a downstream SYNC node,
      // connect it to a downstream SYNC node to
      // preserve a deterministic order. In other words,
      // check if there are invocations of the same reaction across two
      // time steps, if so, connect the previous invocation to the current
      // SYNC node.
      //
      // FIXME: This assumes that the (conventional) completion deadline is the
      // period. We need to find a way to integrate LF deadlines into
      // the picture.
      ArrayList<DagNode> toRemove = new ArrayList<>();
      for (DagNode n : reactionsUnconnectedToSync) {
        if (currentReactions.contains(n.nodeReaction)) {
          dag.addEdge(n, sync);
          toRemove.add(n);
        }
      }
      reactionsUnconnectedToSync.removeAll(toRemove);
      reactionsUnconnectedToSync.addAll(currentReactionNodes);

      // Check if there are invocations of reactions from the same reactor
      // across two time steps. If so, connect invocations from the
      // previous time step to those in the current time step, in order to
      // preserve determinism.
      ArrayList<DagNode> toRemove2 = new ArrayList<>();
      for (DagNode n1 : reactionsUnconnectedToNextInvocation) {
        for (DagNode n2 : currentReactionNodes) {
          ReactorInstance r1 = n1.getReaction().getParent();
          ReactorInstance r2 = n2.getReaction().getParent();
          if (r1.equals(r2)) {
            dag.addEdge(n1, n2);
            toRemove2.add(n1);
          }
        }
      }
      reactionsUnconnectedToNextInvocation.removeAll(toRemove2);
      reactionsUnconnectedToNextInvocation.addAll(currentReactionNodes);

      // Move to the next state space node.
      currentStateSpaceNode = stateSpaceDiagram.getDownstreamNode(currentStateSpaceNode);
      previousSync = sync;
      previousTime = time;
    }

    TimeValue time;
    if (stateSpaceDiagram.isCyclic()) {
      // Set the time of the last SYNC node to be the hyperperiod.
      time = new TimeValue(stateSpaceDiagram.hyperperiod, TimeUnit.NANO);
    } else {
      // Set the time of the last SYNC node to be the tag of the first pending
      // event in the tail node of the state space diagram.
      // Assumption: this assumes that the heap-to-arraylist convertion puts the
      // earliest event in the first location in arraylist.
      if (stateSpaceDiagram.phase == Phase.INIT
        && stateSpaceDiagram.tail.getEventQcopy().size() > 0) {
        time =
            new TimeValue(
                stateSpaceDiagram.tail.getEventQcopy().get(0).getTag().timestamp, TimeUnit.NANO);
      }
      // If there are no pending events, set the time of the last SYNC node to
      // forever. This is just a convention for building DAGs. In reality, we do
      // not want to generate any DU instructions when we see the tail node has
      // TimeValue.MAX_VALUE.
      else time = TimeValue.MAX_VALUE;
    }

    // Add a SYNC node.
    DagNode lastSync = dag.addNode(DagNode.dagNodeType.SYNC, time);
    if (dag.head == null) dag.head = lastSync;

    // Create DUMMY and Connect SYNC and previous SYNC to DUMMY
    if (!time.equals(TimeValue.ZERO)) {
      TimeValue timeDiff = time.sub(previousTime);
      DagNode dummy = dag.addNode(DagNode.dagNodeType.DUMMY, timeDiff);
      dag.addEdge(previousSync, dummy);
      dag.addEdge(dummy, lastSync);
    }

    // Add edges from existing reactions to the last node,
    // and break the loop before adding more reaction nodes.
    for (DagNode n : reactionsUnconnectedToSync) {
      dag.addEdge(n, lastSync);
    }

    // After exiting the while loop, assign the last SYNC node as tail.
    dag.tail = lastSync;

    return dag;
  }
}
