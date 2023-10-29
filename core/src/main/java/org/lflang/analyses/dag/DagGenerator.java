package org.lflang.analyses.dag;

import java.util.ArrayList;
import java.util.PriorityQueue;
import java.util.stream.Collectors;
import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.analyses.statespace.StateSpaceDiagram;
import org.lflang.analyses.statespace.StateSpaceNode;
import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CFileConfig;

/**
 * Constructs a Directed Acyclic Graph (DAG) from the State Space Diagram. This is part of the
 * static schedule generation.
 *
 * <p>FIXME: Currently, there is significant code duplication between generateDagForAcyclicDiagram
 * and generateDagForCyclicDiagram. Redundant code needs to be pruned.
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
    if (stateSpaceDiagram.isCyclic()) {
      return generateDagForCyclicDiagram(stateSpaceDiagram);
    } else {
      return generateDagForAcyclicDiagram(stateSpaceDiagram);
    }
  }

  public Dag generateDagForAcyclicDiagram(StateSpaceDiagram stateSpaceDiagram) {
    // Variables
    Dag dag = new Dag();
    StateSpaceNode currentStateSpaceNode = stateSpaceDiagram.head;
    TimeValue previousTime = TimeValue.ZERO;
    DagNode previousSync = null;
    final TimeValue timeOffset = stateSpaceDiagram.head.getTime();

    // Check if a DAG can be generated for the given state space diagram.
    // Only a diagram without a loop or a loopy diagram without an
    // initialization phase can generate the DAG.

    ArrayList<DagNode> currentReactionNodes = new ArrayList<>();
    ArrayList<DagNode> reactionsUnconnectedToSync = new ArrayList<>();
    ArrayList<DagNode> reactionsUnconnectedToNextInvocation = new ArrayList<>();
    PriorityQueue<TimeValueDagNodeTuple> completionDeadlineQueue = new PriorityQueue<>();

    DagNode sync = null; // Local variable for tracking the current SYNC node.
    while (currentStateSpaceNode != null) {

      // Get the current logical time. Or, if this is the last iteration,
      // set the loop period as the logical time.
      TimeValue time = currentStateSpaceNode.getTime().sub(timeOffset);

      // Deadline Handling Case 1:
      // Check if the head of completionDeadlineQueue is EARLIER than the
      // current time, if so, create a sync node for the completion deadline and
      // connect the previous reaction node.
      if (completionDeadlineQueue.peek() != null
        && completionDeadlineQueue.peek().completionDeadline.compareTo(time) < 0) {

        // Pop the node.
        var cdqHead = completionDeadlineQueue.poll();
        // System.out.println("*** Completion deadline found 1: " + cdqHead.completionDeadline + " for " + cdqHead.reactionNode);

        // Update time to completion deadline time.
        time = cdqHead.completionDeadline;

        // Add a SYNC node.
        sync = dag.addNode(DagNode.dagNodeType.SYNC, time);

        // Create DUMMY and Connect SYNC and previous SYNC to DUMMY
        if (!time.equals(TimeValue.ZERO)) {
          TimeValue timeDiff = time.sub(previousTime);
          DagNode dummy = dag.addNode(DagNode.dagNodeType.DUMMY, timeDiff);
          dag.addEdge(previousSync, dummy);
          dag.addEdge(dummy, sync);
        }

        // Connect the reaction node to the sync node.
        dag.addEdge(cdqHead.reactionNode, sync);

        // Update the variables tracking the previous sync node and time.
        previousSync = sync;
        previousTime = time;

        continue;
      } 

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

        // If the reaction has release deadlines, infer their completion
        // deadlines based on their WCETs.
        if (reaction.declaredDeadline != null) {
          TimeValue releaseDeadline = reaction.declaredDeadline.maxDelay;
          TimeValue completionDeadline = time.add(releaseDeadline).add(reaction.wcet);
          completionDeadlineQueue.add(new TimeValueDagNodeTuple(completionDeadline, node));
          // System.out.println("*** Adding a new reaction to the priority queue: " + reaction + " @ " + completionDeadline);
        }
      }

      // Now add edges based on reaction dependencies.
      for (DagNode n1 : currentReactionNodes) {
        for (DagNode n2 : currentReactionNodes) {
          if (n1.nodeReaction.dependentReactions().contains(n2.nodeReaction)) {
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
      // FIXME: This assumes that the (conventional) deadline is the
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

      // Deadline Handling Case 2:
      // If the current sync node is the deadline completion sync node for some
      // reactions, connect them to the sync node.
      while (completionDeadlineQueue.peek() != null && completionDeadlineQueue.peek().completionDeadline.compareTo(time) == 0) {
        // Pop the node.
        var cdqHead = completionDeadlineQueue.poll();
        // System.out.println("*** Completion deadline found 2: " + cdqHead.completionDeadline + " for " + cdqHead.reactionNode);

        // Connect the reaction node to the sync node.
        dag.addEdge(cdqHead.reactionNode, sync);
      }

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

    // Wrap-up procedure
    wrapup(dag, time, previousSync, previousTime, reactionsUnconnectedToSync, completionDeadlineQueue);

    return dag;
  }

  public Dag generateDagForCyclicDiagram(StateSpaceDiagram stateSpaceDiagram) {
    // Variables
    Dag dag = new Dag();
    StateSpaceNode currentStateSpaceNode = stateSpaceDiagram.head;
    TimeValue previousTime = TimeValue.ZERO;
    DagNode previousSync = null;
    int counter = 0;
    final TimeValue timeOffset = stateSpaceDiagram.head.getTime();

    // Check if a DAG can be generated for the given state space diagram.
    // Only a diagram without a loop or a loopy diagram without an
    // initialization phase can generate the DAG.

    ArrayList<DagNode> currentReactionNodes = new ArrayList<>();
    ArrayList<DagNode> reactionsUnconnectedToSync = new ArrayList<>();
    ArrayList<DagNode> reactionsUnconnectedToNextInvocation = new ArrayList<>();
    PriorityQueue<TimeValueDagNodeTuple> completionDeadlineQueue = new PriorityQueue<>();

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
      TimeValue time = currentStateSpaceNode.getTime().sub(timeOffset);

      // Deadline Handling Case 1:
      // Check if the head of completionDeadlineQueue is EARLIER than the
      // current time, if so, create a sync node for the completion deadline and
      // connect the previous reaction node.
      if (completionDeadlineQueue.peek() != null
        && completionDeadlineQueue.peek().completionDeadline.compareTo(time) < 0) {

        // Pop the node.
        var cdqHead = completionDeadlineQueue.poll();
        // System.out.println("*** Completion deadline found 3: " + cdqHead.completionDeadline + " for " + cdqHead.reactionNode);

        // Update time to completion deadline time.
        time = cdqHead.completionDeadline;

        // Add a SYNC node.
        sync = dag.addNode(DagNode.dagNodeType.SYNC, time);

        // Create DUMMY and Connect SYNC and previous SYNC to DUMMY
        if (!time.equals(TimeValue.ZERO)) {
          TimeValue timeDiff = time.sub(previousTime);
          DagNode dummy = dag.addNode(DagNode.dagNodeType.DUMMY, timeDiff);
          dag.addEdge(previousSync, dummy);
          dag.addEdge(dummy, sync);
        }

        // Connect the reaction node to the sync node.
        dag.addEdge(cdqHead.reactionNode, sync);

        // Update the variables tracking the previous sync node and time.
        previousSync = sync;
        previousTime = time;

        continue;
      } 

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

        // If the reaction has release deadlines, infer their completion
        // deadlines based on their WCETs.
        if (reaction.declaredDeadline != null) {
          TimeValue releaseDeadline = reaction.declaredDeadline.maxDelay;
          TimeValue completionDeadline = time.add(releaseDeadline).add(reaction.wcet);
          completionDeadlineQueue.add(new TimeValueDagNodeTuple(completionDeadline, node));
          // System.out.println("*** Adding a new reaction to the priority queue: " + reaction + " @ " + completionDeadline);
        }
      }

      // Now add edges based on reaction dependencies.
      for (DagNode n1 : currentReactionNodes) {
        for (DagNode n2 : currentReactionNodes) {
          if (n1.nodeReaction.dependentReactions().contains(n2.nodeReaction)) {
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
      // FIXME: This assumes that the (conventional) deadline is the
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

      // Deadline Handling Case 2:
      // If the current sync node is the deadline completion sync node for some
      // reactions, connect them to the sync node.
      while (completionDeadlineQueue.peek() != null && completionDeadlineQueue.peek().completionDeadline.compareTo(time) == 0) {
        // Pop the node.
        var cdqHead = completionDeadlineQueue.poll();
        // System.out.println("*** Completion deadline found 4: " + cdqHead.completionDeadline + " for " + cdqHead.reactionNode);

        // Connect the reaction node to the sync node.
        dag.addEdge(cdqHead.reactionNode, sync);
      }

      // Move to the next state space node.
      currentStateSpaceNode = stateSpaceDiagram.getDownstreamNode(currentStateSpaceNode);
      previousSync = sync;
      previousTime = time;
    }

    // Set the time of the last SYNC node to be the hyperperiod.
    TimeValue endTime = new TimeValue(stateSpaceDiagram.hyperperiod, TimeUnit.NANO);

    // Wrap-up procedure
    wrapup(dag, endTime, previousSync, previousTime, reactionsUnconnectedToSync, completionDeadlineQueue);

    return dag;
  }

  /** A wrap-up procedure */
  private void wrapup(
      Dag dag,
      TimeValue endTime,
      DagNode previousSync,
      TimeValue previousTime,
      ArrayList<DagNode> reactionsUnconnectedToSync,
      PriorityQueue<TimeValueDagNodeTuple> completionDeadlineQueue) {

    // Deadline handling case 3:
    // When we have reach the last node, we check if there are any remaining
    // completion deadline sync nodes to handle. If so, create them.
    while (completionDeadlineQueue.peek() != null
        && completionDeadlineQueue.peek().completionDeadline.compareTo(endTime) < 0) {

      // Pop the node.
      var cdqHead = completionDeadlineQueue.poll();
      // System.out.println("*** Completion deadline found 5: " + cdqHead.completionDeadline + " for " + cdqHead.reactionNode);

      // Update time to completion deadline time.
      var time = cdqHead.completionDeadline;

      // Add a SYNC node.
      DagNode sync = dag.addNode(DagNode.dagNodeType.SYNC, time);

      // Create DUMMY and Connect SYNC and previous SYNC to DUMMY
      if (!time.equals(TimeValue.ZERO)) {
        TimeValue timeDiff = time.sub(previousTime);
        DagNode dummy = dag.addNode(DagNode.dagNodeType.DUMMY, timeDiff);
        dag.addEdge(previousSync, dummy);
        dag.addEdge(dummy, sync);
      }

      // Connect the reaction node to the sync node.
      dag.addEdge(cdqHead.reactionNode, sync);

      // Update the variables tracking the previous sync node and time.
      previousSync = sync;
      previousTime = time;

    } 

    // Add a SYNC node.
    DagNode sync = dag.addNode(DagNode.dagNodeType.SYNC, endTime);
    if (dag.head == null) dag.head = sync;

    // Create DUMMY and Connect SYNC and previous SYNC to DUMMY
    if (!endTime.equals(TimeValue.ZERO)) {
      TimeValue timeDiff = endTime.sub(previousTime);
      DagNode dummy = dag.addNode(DagNode.dagNodeType.DUMMY, timeDiff);
      dag.addEdge(previousSync, dummy);
      dag.addEdge(dummy, sync);
    }

    // Add edges from existing reactions to the last node,
    // and break the loop before adding more reaction nodes.
    for (DagNode n : reactionsUnconnectedToSync) {
      dag.addEdge(n, sync);
    }

    // Deadline Handling Case 2 (again):
    // If the current sync node is the deadline completion sync node for some
    // reactions, connect them to the sync node.
    while (completionDeadlineQueue.peek() != null && completionDeadlineQueue.peek().completionDeadline.compareTo(endTime) == 0) {
      // Pop the node.
      var cdqHead = completionDeadlineQueue.poll();
      // System.out.println("*** Completion deadline found 6: " + cdqHead.completionDeadline + " for " + cdqHead.reactionNode);

      // Connect the reaction node to the sync node.
      dag.addEdge(cdqHead.reactionNode, sync);
    }

    // After exiting the while loop, assign the last SYNC node as tail.
    dag.tail = sync;
  }

  /** Helper inner class for combining a DagNode and a TimeValue */
  class TimeValueDagNodeTuple implements Comparable<TimeValueDagNodeTuple> {
    public final TimeValue completionDeadline;
    public final DagNode reactionNode;

    public TimeValueDagNodeTuple(TimeValue completionDeadline, DagNode reactionNode) {
        this.completionDeadline = completionDeadline;
        this.reactionNode = reactionNode;
    }

    @Override
    public int compareTo(TimeValueDagNodeTuple o) {
      return this.completionDeadline.compareTo(o.completionDeadline);
    }
  }
}
