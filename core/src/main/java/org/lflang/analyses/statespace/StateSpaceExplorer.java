package org.lflang.analyses.statespace;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Set;
import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.generator.ActionInstance;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
import org.lflang.generator.TimerInstance;
import org.lflang.generator.TriggerInstance;
import org.lflang.lf.Expression;
import org.lflang.lf.Time;
import org.lflang.lf.Variable;

/**
 * (EXPERIMENTAL) Explores the state space of an LF program. Use with caution since this is
 * experimental code.
 */
public class StateSpaceExplorer {

  // Instantiate an empty state space diagram.
  public StateSpaceDiagram diagram = new StateSpaceDiagram();

  // Indicate whether a back loop is found in the state space.
  // A back loop suggests periodic behavior.
  public boolean loopFound = false;

  /**
   * Instantiate a global event queue. We will use this event queue to symbolically simulate the
   * logical timeline. This simulation is also valid for runtime implementations that are federated
   * or relax global barrier synchronization, since an LF program defines a unique logical timeline
   * (assuming all reactions behave _consistently_ throughout the execution).
   */
  public EventQueue eventQ = new EventQueue();

  /** The main reactor instance based on which the state space is explored. */
  public ReactorInstance main;

  // Constructor
  public StateSpaceExplorer(ReactorInstance main) {
    this.main = main;
  }

  /** Recursively add the first events to the event queue. */
  public void addInitialEvents(ReactorInstance reactor) {
    // Add the startup trigger, if exists.
    var startup = reactor.getStartupTrigger();
    if (startup != null) eventQ.add(new Event(startup, new Tag(0, 0, false)));

    // Add the initial timer firings, if exist.
    for (TimerInstance timer : reactor.timers) {
      eventQ.add(new Event(timer, new Tag(timer.getOffset().toNanoSeconds(), 0, false)));
    }

    // Recursion
    for (var child : reactor.children) {
      addInitialEvents(child);
    }
  }

  /**
   * Explore the state space and populate the state space diagram until the specified horizon (i.e.
   * the end tag) is reached OR until the event queue is empty.
   *
   * <p>As an optimization, if findLoop is true, the algorithm tries to find a loop in the state
   * space during exploration. If a loop is found (i.e. a previously encountered state is reached
   * again) during exploration, the function returns early.
   *
   * <p>TODOs: 1. Handle action with 0 minimum delay.
   *
   * <p>Note: This is experimental code which is to be refactored in a future PR. Use with caution.
   */
  public void explore(Tag horizon, boolean findLoop) {
    // Traverse the main reactor instance recursively to find
    // the known initial events (startup and timers' first firings).
    // FIXME: It seems that we need to handle shutdown triggers
    // separately, because they could break the back loop.
    addInitialEvents(this.main);

    Tag previousTag = null; // Tag in the previous loop ITERATION
    Tag currentTag = null; // Tag in the current  loop ITERATION
    StateSpaceNode currentNode = null;
    StateSpaceNode previousNode = null;
    HashMap<Integer, StateSpaceNode> uniqueNodes = new HashMap<>();
    boolean stop = true;
    if (this.eventQ.size() > 0) {
      stop = false;
      currentTag = (eventQ.peek()).getTag();
    }

    // A list of reactions invoked at the current logical tag
    Set<ReactionInstance> reactionsInvoked;
    // A temporary list of reactions processed in the current LOOP ITERATION
    Set<ReactionInstance> reactionsTemp;

    while (!stop) {

      // Pop the events from the earliest tag off the event queue.
      ArrayList<Event> currentEvents = new ArrayList<Event>();
      // FIXME: Use stream methods here?
      while (eventQ.size() > 0 && eventQ.peek().getTag().compareTo(currentTag) == 0) {
        Event e = eventQ.poll();
        currentEvents.add(e);
      }

      // Collect all the reactions invoked in this current LOOP ITERATION
      // triggered by the earliest events.
      // Using a hash set here to make sure the reactions invoked
      // are unique. Sometimes multiple events can trigger the same reaction,
      // and we do not want to record duplicate reaction invocations.
      reactionsTemp = new HashSet<ReactionInstance>();
      for (Event e : currentEvents) {
        Set<ReactionInstance> dependentReactions = e.getTrigger().getDependentReactions();
        reactionsTemp.addAll(dependentReactions);

        // If the event is a timer firing, enqueue the next firing.
        if (e.getTrigger() instanceof TimerInstance) {
          TimerInstance timer = (TimerInstance) e.getTrigger();
          eventQ.add(
              new Event(
                  timer,
                  new Tag(
                      e.getTag().timestamp + timer.getPeriod().toNanoSeconds(),
                      0, // A time advancement resets microstep to 0.
                      false)));
        }
      }

      // For each reaction invoked, compute the new events produced.
      for (ReactionInstance reaction : reactionsTemp) {
        // Iterate over all the effects produced by this reaction.
        // If the effect is a port, obtain the downstream port along
        // a connection and enqueue a future event for that port.
        // If the effect is an action, enqueue a future event for
        // this action.
        for (TriggerInstance<? extends Variable> effect : reaction.effects) {
          if (effect instanceof PortInstance) {

            for (SendRange senderRange : ((PortInstance) effect).getDependentPorts()) {

              for (RuntimeRange<PortInstance> destinationRange : senderRange.destinations) {
                PortInstance downstreamPort = destinationRange.instance;

                // Getting delay from connection
                // FIXME: Is there a more concise way to do this?
                long delay = 0;
                Expression delayExpr = senderRange.connection.getDelay();
                if (delayExpr instanceof Time) {
                  long interval = ((Time) delayExpr).getInterval();
                  String unit = ((Time) delayExpr).getUnit();
                  TimeValue timeValue = new TimeValue(interval, TimeUnit.fromName(unit));
                  delay = timeValue.toNanoSeconds();
                }

                // Create and enqueue a new event.
                Event e =
                    new Event(downstreamPort, new Tag(currentTag.timestamp + delay, 0, false));
                eventQ.add(e);
              }
            }
          } else if (effect instanceof ActionInstance) {
            // Get the minimum delay of this action.
            long min_delay = ((ActionInstance) effect).getMinDelay().toNanoSeconds();
            long microstep = 0;
            if (min_delay == 0) {
              microstep = currentTag.microstep + 1;
            }
            // Create and enqueue a new event.
            Event e =
                new Event(effect, new Tag(currentTag.timestamp + min_delay, microstep, false));
            eventQ.add(e);
          }
        }
      }

      // We are at the first iteration.
      // Initialize currentNode.
      if (previousTag == null) {
        //// Now we are done with the node at the previous tag,
        //// work on the new node at the current timestamp.
        // Copy the reactions in reactionsTemp.
        reactionsInvoked = new HashSet<ReactionInstance>(reactionsTemp);

        // Create a new state in the SSD for the current tag,
        // add the reactions triggered to the state,
        // and add a snapshot of the event queue (with new events
        // generated by reaction invocations in the curren tag)
        // to the state.
        StateSpaceNode node =
            new StateSpaceNode(
                currentTag, // Current tag
                reactionsInvoked, // Reactions invoked at this tag
                new ArrayList<Event>(eventQ) // A snapshot of the event queue
                );

        // Initialize currentNode.
        currentNode = node;
      }
      // When we advance to a new TIMESTAMP (not a new tag),
      // create a new node in the state space diagram
      // for everything processed in the previous timestamp.
      // This makes sure that the granularity of nodes is
      // at the timestamp-level, so that we don't have to
      // worry about microsteps.
      else if (previousTag != null && currentTag.timestamp > previousTag.timestamp) {
        // Whenever we finish a tag, check for loops fist.
        // If currentNode matches an existing node in uniqueNodes,
        // duplicate is set to the existing node.
        StateSpaceNode duplicate;
        if (findLoop && (duplicate = uniqueNodes.put(currentNode.hash(), currentNode)) != null) {

          // Mark the loop in the diagram.
          loopFound = true;
          this.diagram.loopNode = duplicate;
          this.diagram.loopNodeNext = currentNode;
          this.diagram.tail = previousNode;
          // Loop period is the time difference between the 1st time
          // the node is reached and the 2nd time the node is reached.
          this.diagram.loopPeriod =
              this.diagram.loopNodeNext.getTag().timestamp
                  - this.diagram.loopNode.getTag().timestamp;
          this.diagram.addEdge(this.diagram.loopNode, this.diagram.tail);
          return; // Exit the while loop early.
        }

        // Now we are at a new tag, and a loop is not found,
        // add the node to the state space diagram.
        // Adding a node to the graph once it is finalized
        // because this makes checking duplicate nodes easier.
        // We don't have to remove a node from the graph.
        this.diagram.addNode(currentNode);
        this.diagram.tail = currentNode; // Update the current tail.

        // If the head is not empty, add an edge from the previous state
        // to the next state. Otherwise initialize the head to the new node.
        if (previousNode != null) {
          // System.out.println("--- Add a new edge between " + currentNode + " and " + node);
          // this.diagram.addEdge(currentNode, previousNode); // Sink first, then source
          if (previousNode != currentNode) this.diagram.addEdge(currentNode, previousNode);
        } else this.diagram.head = currentNode; // Initialize the head.

        //// Now we are done with the node at the previous tag,
        //// work on the new node at the current timestamp.
        // Copy the reactions in reactionsTemp.
        reactionsInvoked = new HashSet<ReactionInstance>(reactionsTemp);

        // Create a new state in the SSD for the current tag,
        // add the reactions triggered to the state,
        // and add a snapshot of the event queue (with new events
        // generated by reaction invocations in the curren tag)
        // to the state.
        StateSpaceNode node =
            new StateSpaceNode(
                currentTag, // Current tag
                reactionsInvoked, // Reactions invoked at this tag
                new ArrayList<Event>(eventQ) // A snapshot of the event queue
                );

        // Update the previous node.
        previousNode = currentNode;
        // Update the current node to the new (potentially incomplete) node.
        currentNode = node;
      }
      // Timestamp does not advance because we are processing
      // connections with zero delay.
      else if (previousTag != null && currentTag.timestamp == previousTag.timestamp) {
        // Add reactions explored in the current loop iteration
        // to the existing state space node.
        currentNode.getReactionsInvoked().addAll(reactionsTemp);
        // Update the eventQ snapshot.
        currentNode.setEventQcopy(new ArrayList<Event>(eventQ));
      } else {
        throw new AssertionError("Unreachable");
      }

      // Update the current tag for the next iteration.
      if (eventQ.size() > 0) {
        previousTag = currentTag;
        currentTag = eventQ.peek().getTag();
      }

      // Stop if:
      // 1. the event queue is empty, or
      // 2. the horizon is reached.
      if (eventQ.size() == 0) {
        stop = true;
      } else if (currentTag.timestamp > horizon.timestamp) {
        stop = true;
      }
    }

    // Check if the last current node is added to the graph yet.
    // If not, add it now.
    // This could happen when condition (previousTag == null)
    // or (previousTag != null
    // && currentTag.compareTo(previousTag) > 0) is true and then
    // the simulation ends, leaving a new node dangling.
    if (previousNode == null || previousNode.getTag().timestamp < currentNode.getTag().timestamp) {
      this.diagram.addNode(currentNode);
      this.diagram.tail = currentNode; // Update the current tail.
      if (previousNode != null) {
        this.diagram.addEdge(currentNode, previousNode);
      }
    }

    // When we exit and we still don't have a head,
    // that means there is only one node in the diagram.
    // Set the current node as the head.
    if (this.diagram.head == null) this.diagram.head = currentNode;

    return;
  }
}
