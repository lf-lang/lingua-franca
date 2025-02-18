package org.lflang.pretvm.dag;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.PriorityQueue;
import java.util.Set;
import java.util.stream.Collectors;
import org.lflang.TimeValue;
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.analyses.statespace.StateSpaceNode;
import org.lflang.generator.DeadlineInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CFileConfig;
import org.lflang.pretvm.ExecutionPhase;
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
 * the state space diagram, and manages variables like logical time and time node nodes to track the
 * flow of execution. - Various lists are used to track unconnected reaction nodes for processing
 * later.
 *
 * <p>2. **time node Node Creation**: - For each node in the state space, a time node is added to
 * the DAG to represent the logical time of that state. If it's not the first time node, a "dummy"
 * node is created to account for the time difference between time node nodes and to ensure the
 * correct order of execution.
 *
 * <p>3. **Reaction Nodes**: - Reactions invoked at the current state are added to the DAG as
 * reaction nodes. These nodes are connected to the time node, marking the time when the reactions
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
 * <p>8. **Final time node Node**: - After all nodes in the state space diagram are processed, a
 * final time node is added. This node represents the logical time at which the last event or state
 * transition occurs in the diagram.
 *
 * <p>9. **Completion**: - The DAG is finalized by adding edges from any remaining unconnected
 * reaction nodes to the last time node. This ensures all nodes are correctly linked, and the last
 * time node is marked as the tail of the DAG.
 *
 * <p>The result is a time-sensitive DAG that respects logical dependencies, time constraints, and
 * priority rules, enabling deterministic execution of the reactor system.
 *
 * <p>=========================================
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
    /////// Variables
    // The Directed Acyclic Graph (DAG) being constructed, which will
    // represent the static schedule.
    Dag dag = new Dag();
    // The current node being processed in the state space diagram. It
    // starts with the head of the state space.
    StateSpaceNode currentStateSpaceNode = stateSpaceDiagram.head;
    // The time offset for normalizing the time values across the state
    // space diagram. It is initialized to the time of the first state
    // space node, so that the DAG's first time node always start at t=0.
    TimeValue timeOffset = stateSpaceDiagram.head.getTime();
    // A counter to track how many times the loop node has been
    // encountered in cyclic state space diagrams. It is used to
    // terminate the loop correctly.
    // Only used when the diagram is cyclic.
    int loopNodeCounter = 0;
    // A list to store the reaction nodes that are invoked at the
    // current state space node and will be connected to the current
    // time node.
    List<JobNode> currentJobNodes = new ArrayList<>();
    // A list to hold DagNode objects representing reactions that are
    // not connected to any time node.
    List<JobNode> jobsUnconnectedToSync = new ArrayList<>();
    // A list to hold reaction nodes that need to be connected to future
    // invocations of the same reaction across different time steps.
    List<JobNode> jobsUnconnectedToNextInvocation = new ArrayList<>();
    // A priority queue for sorting all time nodes based on timestamp.
    PriorityQueue<TimeNode> timeNodesPQueue = new PriorityQueue<TimeNode>();
    // A set of reaction nodes with deadlines
    Set<JobNode> jobNodesWithDeadlines = new HashSet<>();
    // Local variable for tracking the current time node.
    TimeNode timeNode = null;

    // A map used to track unconnected upstream DAG nodes for reaction
    // invocations. For example, when we encounter DAG node N_A (for reaction A
    // invoked at t=0), and from the LF program, we know that A could send an
    // event to B with a 10 msec delay and another event to C with a 20 msec
    // delay, in this map, we will have two entries {(B, 10 msec) -> N_A, (C, 20
    // msec) -> N_A}. When we later visit a DAG node N_X that matches any of the
    // key, we can draw an edge N_A -> N_X.
    // The map value is a DagNode list because multiple upstream dag nodes can
    // be looking for the same node matching the <reaction, time> criteria.
    Map<Pair<ReactionInstance, TimeValue>, List<JobNode>> unconnectedUpstreamDagNodes =
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

      // Add a time node.
      timeNode = addTimeNodeToDag(dag, time, timeNodesPQueue);
      if (dag.start == null) dag.start = timeNode;

      // Add reaction nodes, as well as the edges connecting them to time node.
      currentJobNodes.clear();
      for (ReactionInstance reaction : currentStateSpaceNode.getReactionsInvoked()) {
        JobNode reactionNode = new JobNode(reaction);
        dag.addNode(reactionNode);
        currentJobNodes.add(reactionNode);
        dag.addEdge(timeNode, reactionNode);
        reactionNode.setAssociatedSyncNode(timeNode);
        // If the reaction has a deadline, add it to the set.
        if (reaction.declaredDeadline != null) jobNodesWithDeadlines.add(reactionNode);
      }

      // Add edges based on reaction priorities.
      for (JobNode n1 : currentJobNodes) {
        for (JobNode n2 : currentJobNodes) {
          // Add an edge for reactions in the same reactor based on priorities.
          // This adds the remaining dependencies not accounted for in
          // dependentReactions(), e.g., reaction 3 depends on reaction 1 in the
          // same reactor.
          if (n1.getReaction().getParent() == n2.getReaction().getParent()
              && n1.getReaction().index < n2.getReaction().index) {
            dag.addEdge(n1, n2);
          }
        }
      }

      // Update the unconnectedUpstreamDagNodes map.
      for (JobNode reactionNode : currentJobNodes) {
        ReactionInstance reaction = reactionNode.getReaction();
        var downstreamReactionsMap = reaction.downstreamReactions();
        for (var entry : downstreamReactionsMap.entrySet()) {
          ReactionInstance downstreamReaction = entry.getKey();
          Long expectedTime = entry.getValue() + time.toNanoSeconds();
          TimeValue expectedTimeValue = TimeValue.fromNanoSeconds(expectedTime);
          Pair<ReactionInstance, TimeValue> pair =
              new Pair<ReactionInstance, TimeValue>(downstreamReaction, expectedTimeValue);
          // Check if the value is empty.
          List<JobNode> list = unconnectedUpstreamDagNodes.get(pair);
          if (list == null)
            unconnectedUpstreamDagNodes.put(pair, new ArrayList<>(Arrays.asList(reactionNode)));
          else list.add(reactionNode);
        }
      }
      // Add edges based on connections (including the delayed ones)
      // using unconnectedUpstreamDagNodes.
      for (JobNode jobNode : currentJobNodes) {
        ReactionInstance reaction = jobNode.getReaction();
        var searchKey = new Pair<ReactionInstance, TimeValue>(reaction, time);
        List<JobNode> upstreams = unconnectedUpstreamDagNodes.get(searchKey);
        if (upstreams != null) {
          for (JobNode us : upstreams) {
            dag.addEdge(us, jobNode);
            dag.addWUDependency(jobNode, us);
          }
        }
      }

      // Create a list of ReactionInstances from currentJobNodes.
      ArrayList<ReactionInstance> currentReactions =
          currentJobNodes.stream()
              .map(JobNode::getReaction)
              .collect(Collectors.toCollection(ArrayList::new));

      // If there is a newly released reaction found and its prior
      // invocation is not connected to a downstream time node,
      // connect it to a downstream time node to
      // preserve a deterministic order. In other words,
      // check if there are invocations of the same reaction across two
      // time steps, if so, connect the previous invocation to the current
      // time node.
      //
      // FIXME: This assumes that the (conventional) completion deadline is the
      // period. We need to find a way to integrate LF deadlines into
      // the picture.
      ArrayList<DagNode> toRemove = new ArrayList<>();
      for (JobNode n : jobsUnconnectedToSync) {
        if (currentReactions.contains(n.getReaction())) {
          dag.addEdge(n, timeNode);
          toRemove.add(n);
        }
      }
      jobsUnconnectedToSync.removeAll(toRemove);
      jobsUnconnectedToSync.addAll(currentJobNodes);

      // Check if there are invocations of reactions from the same reactor
      // across two time steps. If so, connect invocations from the
      // previous time step to those in the current time step, in order to
      // preserve determinism.
      ArrayList<DagNode> toRemove2 = new ArrayList<>();
      for (JobNode n1 : jobsUnconnectedToNextInvocation) {
        for (JobNode n2 : currentJobNodes) {
          ReactorInstance r1 = n1.getReaction().getParent();
          ReactorInstance r2 = n2.getReaction().getParent();
          if (r1.equals(r2)) {
            dag.addEdge(n1, n2);
            toRemove2.add(n1);
          }
        }
      }
      jobsUnconnectedToNextInvocation.removeAll(toRemove2);
      jobsUnconnectedToNextInvocation.addAll(currentJobNodes);

      // Move to the next state space node.
      currentStateSpaceNode = stateSpaceDiagram.getDownstreamNode(currentStateSpaceNode);
    }

    TimeValue time;
    if (stateSpaceDiagram.isCyclic()) {
      // Set the time of the last time node to be the hyperperiod.
      time = TimeValue.fromNanoSeconds(stateSpaceDiagram.hyperperiod);
    } else {
      // Set the time of the last time node to be the tag of the first pending
      // event in the tail node of the state space diagram.
      // Assumption: this assumes that the heap-to-arraylist convertion puts the
      // earliest event in the first location in arraylist.
      if (stateSpaceDiagram.phase == ExecutionPhase.INIT
          && stateSpaceDiagram.tail.getEventQcopy().size() > 0) {
        time =
            TimeValue.fromNanoSeconds(
                stateSpaceDiagram.tail.getEventQcopy().get(0).getTag().time.toNanoSeconds());
      }
      // If there are no pending events, set the time of the last time node to
      // forever. This is just a convention for building DAGs. In reality, we do
      // not want to generate any DU instructions when we see the tail node has
      // TimeValue.MAX_VALUE.
      else time = TimeValue.MAX_VALUE;
    }

    // Add a time node when (1) the state space is cyclic and we
    // encounter the loop node for the 2nd time
    // or (2) the state space is a chain and we are at the end of the
    // end of the chain.
    timeNode = addTimeNodeToDag(dag, time, timeNodesPQueue);
    // If we still don't have a head node at this point, make it the
    // head node. This might happen when a reactor has no reactions.
    // FIXME: Double check if this is the case.
    if (dag.start == null) dag.start = timeNode;
    // This timeNode node is also the end of the hyperperiod / DAG task set.
    dag.end = timeNode;

    // Add edges from existing reactions to the last node.
    for (DagNode n : jobsUnconnectedToSync) {
      dag.addEdge(n, timeNode);
    }

    ///// Deadline handling /////
    // For each reaction that has a deadline, create a time node and
    // point the reaction node to the time node.
    for (JobNode reactionNode : jobNodesWithDeadlines) {
      // The associated time node contains the timestamp at which the
      // reaction is invoked. We add the deadline value to the timestamp
      // to get the deadline time.
      ReactionInstance reaction = reactionNode.getReaction();
      DeadlineInstance deadline = reaction.declaredDeadline;
      TimeValue deadlineValue = deadline.maxDelay;
      TimeNode associatedSync = reactionNode.getAssociatedSyncNode();
      // FIXME: We need a cleaner DAG formalism so that we stop
      // modeling release deadlines as completion deadlines.
      TimeValue reactionWcet = reaction.wcet;
      // Modeling the release deadline as a completion deadline.
      // Completion deadline = release time + WCET + deadline value.
      TimeValue deadlineTime = associatedSync.getTime().add(reactionWcet).add(deadlineValue);
      // Check if a time node with the same time already exists.
      // Skip the node creation steps if a node with the same timestep
      // exists.
      Optional<DagNode> syncNodeWithSameTime =
          dag.dagNodes.stream()
              .filter(node -> node instanceof TimeNode tn && tn.getTime().equals(deadlineTime))
              .findFirst();
      TimeNode timeNodeToAdd; // The time node to be added.
      // If a time node with the same time exists, use it.
      if (syncNodeWithSameTime.isPresent()) {
        timeNodeToAdd = (TimeNode) syncNodeWithSameTime.get();
      } else {
        // Otherwise, create and add a time node inferred from the deadline.
        timeNodeToAdd = addTimeNodeToDag(dag, deadlineTime, timeNodesPQueue);
      }
      // Add an edge from the reaction node to the time node.
      dag.addEdge(reactionNode, timeNodeToAdd);
    }
    /////////////////////////////

    // At this point, all time node nodes should have been generated.
    // Sort the time node nodes based on their time steps by polling from the
    // priority queue.
    TimeNode upstreamTimeNode = null;
    TimeNode downstreamTimeNode = null;
    while (!timeNodesPQueue.isEmpty()) {
      // The previous downstream time node becomes the upstream time node.
      upstreamTimeNode = downstreamTimeNode;
      // The next downstream time node is the next node in the priority queue.
      downstreamTimeNode = timeNodesPQueue.poll();
      // Add dummy nodes between every pair of time node nodes.
      if (upstreamTimeNode != null)
        createDummyNodeBetweenTwoSyncNodes(dag, upstreamTimeNode, downstreamTimeNode);
    }

    // assign the last time node as tail.
    // FIXME: This is probably not used anywhere.
    // The more useful node is the end node.
    dag.tail = downstreamTimeNode;

    return dag;
  }

  /**
   * Create a DUMMY node and place it between an upstream time node and a downstream time node.
   *
   * @param dag The DAG to be updated
   * @param upstreamTimeNode The time node with an earlier tag
   * @param downstreamTimeNode The time node with a later tag
   */
  private void createDummyNodeBetweenTwoSyncNodes(
      Dag dag, TimeNode upstreamTimeNode, TimeNode downstreamTimeNode) {
    TimeValue timeDiff = downstreamTimeNode.getTime().subtract(upstreamTimeNode.getTime());
    IntervalNode intervalNode = new IntervalNode(timeDiff);
    dag.addNode(intervalNode);
    dag.addEdge(upstreamTimeNode, intervalNode);
    dag.addEdge(intervalNode, downstreamTimeNode);
  }

  /**
   * Helper function for adding a time node to a DAG.
   *
   * @param dag The DAG to be updated
   * @param time The timestamp of the time node
   * @param timeNodesPQueue The priority queue to add the timeNode node to
   * @return a newly created time node
   */
  private TimeNode addTimeNodeToDag(
      Dag dag, TimeValue time, PriorityQueue<TimeNode> timeNodesPQueue) {
    TimeNode timeNode = new TimeNode(time);
    dag.addNode(timeNode);
    timeNodesPQueue.add(timeNode); // Track the node in the priority queue.
    return timeNode;
  }
}
