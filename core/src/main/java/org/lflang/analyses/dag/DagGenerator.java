package org.lflang.analyses.dag;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;
import org.lflang.analyses.statespace.StateSpaceNode;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CFileConfig;
import org.lflang.util.Pair;

/**
 * Constructs a Directed Acyclic Graph (DAG) from the State Space Diagram. This is part of the
 * static schedule generation.
 *
 * <p>FIXME: DAG generation does not need to be stateful. The methods in this class can be
 * refactored into static methods.
 *
 * <p>=========== Algorithm Summary ===========
 *
 * <p>The DagGenerator class is responsible for creating a Directed Acyclic Graph (DAG) from a given
 * state space diagram. The primary purpose of this DAG is to represent the static schedule of
 * reactions in a reactor-based program, considering their logical time, dependencies, and
 * priorities.
 *
 * <p>Key Steps in the DAG Generation:
 *
 * <p>1. **Initialization**: - The generator initializes a DAG structure, sets up the head node of
 * the state space diagram, and manages variables like logical time and SYNC nodes to track the flow
 * of execution. - Various lists are used to track unconnected reaction nodes for processing later.
 *
 * <p>2. **SYNC Node Creation**: - For each node in the state space, a SYNC node is added to the DAG
 * to represent the logical time of that state. If it's not the first SYNC node, a "dummy" node is
 * created to account for the time difference between SYNC nodes and to ensure the correct order of
 * execution.
 *
 * <p>3. **Reaction Nodes**: - Reactions invoked at the current state are added to the DAG as
 * reaction nodes. These nodes are connected to the SYNC node, marking the time when the reactions
 * are triggered.
 *
 * <p>4. **Priority-based Edges**: - Edges between reaction nodes are created based on their
 * priorities. This step ensures the correct order of execution for reactions within the same
 * reactor, according to their priority levels.
 *
 * <p>5. **Data Dependencies**: - The generator tracks dependencies between reactions, including
 * those with delays. It maintains a map of unconnected upstream reaction nodes, which are later
 * connected when the corresponding downstream reactions are encountered at the appropriate logical
 * time.
 *
 * <p>6. **Cross-Time Dependencies of the Same Reaction**: - To maintain determinism across time
 * steps, the generator connects reactions that are invoked over multiple time steps. This includes
 * adding edges between earlier and current invocations of the same reaction.
 *
 * <p>7. **Loop Detection and Stop Conditions**: - If the state space diagram is cyclic, the
 * algorithm detects when the loop has been completed by revisiting the loop node. It terminates the
 * processing after encountering the loop node a second time.
 *
 * <p>8. **Final SYNC Node**: - After all nodes in the state space diagram are processed, a final
 * SYNC node is added. This node represents the logical time at which the last event or state
 * transition occurs in the diagram.
 *
 * <p>9. **Completion**: - The DAG is finalized by adding edges from any remaining unconnected
 * reaction nodes to the last SYNC node. This ensures all nodes are correctly linked, and the last
 * SYNC node is marked as the tail of the DAG.
 *
 * <p>The result is a time-sensitive DAG that respects logical dependencies, time constraints, and
 * priority rules, enabling deterministic execution of the reactor system.
 *
 * <p>=========================================
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
    // System.out.println("Generating DAG for " + stateSpaceDiagram.phase);
    // Variables
    Dag dag = new Dag();
    StateSpaceNode currentStateSpaceNode = stateSpaceDiagram.head;
    TimeValue previousTime = TimeValue.ZERO;
    DagNode previousSync = null;
    final TimeValue timeOffset = stateSpaceDiagram.head.getTime();
    int loopNodeCounter = 0; // Only used when the diagram is cyclic.
    ArrayList<DagNode> currentReactionNodes = new ArrayList<>();
    // FIXME: Need documentation.
    ArrayList<DagNode> reactionsUnconnectedToSync = new ArrayList<>();
    // FIXME: Need documentation.
    ArrayList<DagNode> reactionsUnconnectedToNextInvocation = new ArrayList<>();
    DagNode sync = null; // Local variable for tracking the current SYNC node.

    // A map used to track unconnected upstream DAG nodes for reaction
    // invocations. For example, when we encounter DAG node N_A (for reaction A
    // invoked at t=0), and from the LF program, we know that A could send an
    // event to B with a 10 msec delay and another event to C with a 20 msec
    // delay, in this map, we will have two entries {(B, 10 msec) -> N_A, (C, 20
    // msec) -> N_A}. When we later visit a DAG node N_X that matches any of the
    // key, we can draw an edge N_A -> N_X.
    // The map value is a DagNode list because multiple upstream dag nodes can
    // be looking for the same node matching the <reaction, time> criteria.
    Map<Pair<ReactionInstance, TimeValue>, List<DagNode>> unconnectedUpstreamDagNodes =
        new HashMap<>();

    // FIXME: Check if a DAG can be generated for the given state space diagram.
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
      TimeValue time = currentStateSpaceNode.getTime().subtract(timeOffset);

      // Add a SYNC node.
      sync = dag.addNode(DagNode.dagNodeType.SYNC, time);
      if (dag.head == null) dag.head = sync;

      // Create DUMMY and Connect SYNC and previous SYNC to DUMMY
      if (!time.equals(TimeValue.ZERO)) {
        TimeValue timeDiff = time.subtract(previousTime);
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

      // Add edges based on reaction priorities.
      for (DagNode n1 : currentReactionNodes) {
        for (DagNode n2 : currentReactionNodes) {
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

      // Update the unconnectedUpstreamDagNodes map.
      for (DagNode reactionNode : currentReactionNodes) {
        ReactionInstance reaction = reactionNode.nodeReaction;
        var downstreamReactionsSet = reaction.downstreamReactions();
        for (var pair : downstreamReactionsSet) {
          ReactionInstance downstreamReaction = pair.first();
          Long expectedTime = pair.second() + time.toNanoSeconds();
          TimeValue expectedTimeValue = TimeValue.fromNanoSeconds(expectedTime);
          Pair<ReactionInstance, TimeValue> _pair =
              new Pair<ReactionInstance, TimeValue>(downstreamReaction, expectedTimeValue);
          // Check if the value is empty.
          List<DagNode> list = unconnectedUpstreamDagNodes.get(_pair);
          if (list == null)
            unconnectedUpstreamDagNodes.put(_pair, new ArrayList<>(Arrays.asList(reactionNode)));
          else list.add(reactionNode);
          // System.out.println(reactionNode + " looking for: " + downstreamReaction + " @ " +
          // expectedTimeValue);
        }
      }
      // Add edges based on connections (including the delayed ones)
      // using unconnectedUpstreamDagNodes.
      for (DagNode reactionNode : currentReactionNodes) {
        ReactionInstance reaction = reactionNode.nodeReaction;
        var searchKey = new Pair<ReactionInstance, TimeValue>(reaction, time);
        // System.out.println("Search key: " + reaction + " @ " + time);
        List<DagNode> upstreams = unconnectedUpstreamDagNodes.get(searchKey);
        if (upstreams != null) {
          for (DagNode us : upstreams) {
            dag.addEdge(us, reactionNode);
            dag.addWUDependency(reactionNode, us);
            // System.out.println("Match!");
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
      TimeValue timeDiff = time.subtract(previousTime);
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
