package org.lflang.analyses.statespace;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.ActionInstance;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
import org.lflang.generator.TimerInstance;
import org.lflang.generator.TriggerInstance;
import org.lflang.lf.Expression;
import org.lflang.lf.Variable;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.TimeOutProperty;

/**
 * (EXPERIMENTAL) Explores the state space of an LF program. Use with caution since this is
 * experimental code.
 *
 * @author Shaokai Lin
 */
public class StateSpaceExplorer {

  /**
   * Common phases of a logical timeline, some of which are provided to the explorer as directives.
   */
  public enum Phase {
    PREAMBLE,
    INIT, // Dominated by startup triggers and initial timer firings
    PERIODIC,
    EPILOGUE,
    SYNC_BLOCK,
    INIT_AND_PERIODIC,
    SHUTDOWN_TIMEOUT,
    SHUTDOWN_STARVATION,
    ASYNC, // Dominated by physical actions
  }

  /** Target configuration */
  TargetConfig targetConfig;

  /** Constructor */
  public StateSpaceExplorer(TargetConfig targetConfig) {
    this.targetConfig = targetConfig;
  }

  /**
   * Explore the state space and populate the state space diagram until the specified horizon (i.e.
   * the end tag) is reached OR until the event queue is empty.
   *
   * <p>As an optimization, the algorithm tries to find a loop in the state space during
   * exploration. If a loop is found (i.e. a previously encountered state is reached again) during
   * exploration, the function returns early.
   *
   * <p>If the phase is INIT_AND_PERIODIC, the explorer starts with startup triggers and timers'
   * initial firings. If the phase is SHUTDOWN_*, the explorer starts with shutdown triggers.
   *
   * <p>TODOs: 1. Handle action with 0 minimum delay. 2. Handle hierarchical reactors.
   *
   * <p>Note: This is experimental code. Use with caution.
   */
  public StateSpaceDiagram explore(
      ReactorInstance main, Tag horizon, Phase phase, List<Event> initialEvents) {
    if (!(phase == Phase.INIT_AND_PERIODIC
        || phase == Phase.SHUTDOWN_TIMEOUT
        || phase == Phase.SHUTDOWN_STARVATION
        || phase == Phase.ASYNC))
      throw new RuntimeException("Unsupported phase detected in the explorer.");

    // Variable initilizations
    StateSpaceDiagram diagram = new StateSpaceDiagram();
    diagram.phase = phase;
    EventQueue eventQ = new EventQueue();
    Tag previousTag = null; // Tag in the previous loop ITERATION
    Tag currentTag = null; // Tag in the current  loop ITERATION
    StateSpaceNode currentNode = null;
    StateSpaceNode previousNode = null;
    HashMap<Integer, StateSpaceNode> uniqueNodes = new HashMap<>();
    boolean stop = true;

    // Add initial events to the event queue.
    eventQ.addAll(initialEvents);

    // Set appropriate fields if the phase is ASYNC.
    if (phase == Phase.ASYNC) {
      if (eventQ.size() != 1)
        throw new RuntimeException(
            "When exploring the ASYNC phase, there should be only ONE initial event at a time."
                + " eventQ.size() = "
                + eventQ.size());
      diagram.makeAsync();
      diagram.setMinSpacing(((ActionInstance) eventQ.peek().getTrigger()).getMinSpacing());
    }

    // Check if we should stop already.
    if (eventQ.size() > 0) {
      stop = false;
      currentTag = eventQ.peek().getTag();
    }

    // A list of reactions invoked at the current logical tag
    Set<ReactionInstance> reactionsInvoked;
    // A temporary list of reactions processed in the current LOOP ITERATION
    Set<ReactionInstance> reactionsTemp;

    // Iterate until stop conditions are met.
    while (!stop) {

      // Pop the events from the earliest tag off the event queue.
      List<Event> currentEvents = popCurrentEvents(eventQ, currentTag);

      // Collect all the reactions invoked in this current LOOP ITERATION
      // triggered by the earliest events.
      reactionsTemp = getReactionsTriggeredByCurrentEvents(currentEvents);

      // For each reaction invoked, compute the new events produced.
      List<Event> newEvents = createNewEvents(currentEvents, reactionsTemp, currentTag);
      // FIXME: Need to make sure that addAll() is using the overridden version
      // that makes sure new events added are unique. By default, this should be
      // the case.
      eventQ.addAll(newEvents);

      // We are at the first iteration.
      // Initialize currentNode.
      if (previousTag == null) {
        //// Now we are done with the node at the previous tag,
        //// work on the new node at the current timestamp.
        // Copy the reactions in reactionsTemp.
        reactionsInvoked = new HashSet<>(reactionsTemp);

        // Create a new state in the SSD for the current tag,
        // add the reactions triggered to the state,
        // and add a snapshot of the event queue (with new events
        // generated by reaction invocations in the curren tag)
        // to the state.

        // Initialize currentNode.
        currentNode =
            new StateSpaceNode(
                currentTag, // Current tag
                reactionsInvoked, // Reactions invoked at this tag
                new ArrayList<>(eventQ) // A snapshot of the event queue
                );
      }
      // When we advance to a new TIMESTAMP (not a new tag),
      // create a new node in the state space diagram
      // for everything processed in the previous timestamp.
      // This makes sure that the granularity of nodes is
      // at the timestamp-level, so that we don't have to
      // worry about microsteps.
      else if (previousTag != null && currentTag.timestamp > previousTag.timestamp) {
        // Check if we are in the SHUTDOWN_TIMEOUT mode,
        // if so, stop the loop immediately, because TIMEOUT is the last tag.
        if (phase == Phase.SHUTDOWN_TIMEOUT) {
          // Make the hyperperiod for the SHUTDOWN_TIMEOUT phase Long.MAX_VALUE,
          // so that this is guaranteed to be feasibile from the perspective of
          // the EGS scheduler.
          diagram.hyperperiod = Long.MAX_VALUE;
          diagram.loopNode = null; // The SHUTDOWN_TIMEOUT phase is acyclic.
          break;
        }

        // Whenever we finish a tag, check for loops fist.
        // If currentNode matches an existing node in uniqueNodes,
        // duplicate is set to the existing node.
        StateSpaceNode duplicate;
        if ((duplicate = uniqueNodes.put(currentNode.hash(), currentNode)) != null) {

          // Mark the loop in the diagram.
          diagram.loopNode = duplicate;
          diagram.loopNodeNext = currentNode;
          diagram.tail = previousNode;
          // Loop period is the time difference between the 1st time
          // the node is reached and the 2nd time the node is reached.
          diagram.hyperperiod =
              diagram.loopNodeNext.getTag().timestamp - diagram.loopNode.getTag().timestamp;
          diagram.addEdge(diagram.loopNode, diagram.tail);
          return diagram; // Exit the while loop early.
        }

        // Now we are at a new tag, and a loop is not found,
        // add the node to the state space diagram.
        // Adding a node to the graph once it is finalized
        // because this makes checking duplicate nodes easier.
        // We don't have to remove a node from the graph.
        diagram.addNode(currentNode);
        diagram.tail = currentNode; // Update the current tail.

        // If the head is not empty, add an edge from the previous state
        // to the next state. Otherwise initialize the head to the new node.
        if (previousNode != null) {
          if (previousNode != currentNode) diagram.addEdge(currentNode, previousNode);
        } else diagram.head = currentNode; // Initialize the head.

        //// Now we are done with the node at the previous tag,
        //// work on the new node at the current timestamp.
        // Copy the reactions in reactionsTemp.
        reactionsInvoked = new HashSet<>(reactionsTemp);

        // Create a new state in the SSD for the current tag,
        // add the reactions triggered to the state,
        // and add a snapshot of the event queue (with new events
        // generated by reaction invocations in the curren tag)
        // to the state.
        StateSpaceNode node =
            new StateSpaceNode(
                currentTag, // Current tag
                reactionsInvoked, // Reactions invoked at this tag
                new ArrayList<>(eventQ) // A snapshot of the event queue
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
        currentNode.setEventQcopy(new ArrayList<>(eventQ));
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
      }
      // FIXME: If horizon is forever, explore() might not terminate.
      // How to set a reasonable upperbound?
      else if (!horizon.forever && currentTag.timestamp > horizon.timestamp) {
        stop = true;
      }
    }

    // Check if the last current node is added to the graph yet.
    // If not, add it now.
    // This could happen when condition (previousTag == null)
    // or (previousTag != null
    // && currentTag.compareTo(previousTag) > 0) is true and then
    // the simulation ends, leaving a new node dangling.
    if (currentNode != null
        && (previousNode == null
            || previousNode.getTag().timestamp < currentNode.getTag().timestamp)) {
      diagram.addNode(currentNode);
      diagram.tail = currentNode; // Update the current tail.
      if (previousNode != null) {
        diagram.addEdge(currentNode, previousNode);
      }
    }

    // At this point if we still don't have a head,
    // then it means there is only one node in the diagram.
    // Set the current node as the head.
    if (diagram.head == null) diagram.head = currentNode;

    return diagram;
  }

  //////////////////////////////////////////////////////
  ////////////////// Private Methods

  /**
   * Return a (unordered) list of initial events to be given to the state space explorer based on a
   * given phase.
   *
   * @param reactor The reactor wrt which initial events are inferred
   * @param phase The phase for which initial events are inferred
   * @return A list of initial events
   */
  public static List<Event> addInitialEvents(
      ReactorInstance reactor, Phase phase, TargetConfig targetConfig) {
    List<Event> events = new ArrayList<>();
    addInitialEventsRecursive(reactor, events, phase, targetConfig);
    return events;
  }

  /**
   * Recursively add the first events to the event list for state space exploration. For the
   * SHUTDOWN modes, it is okay to create shutdown events at (0,0) because this tag is a relative
   * offset wrt to a phase (e.g., the shutdown phase), not the absolute tag at runtime.
   */
  public static void addInitialEventsRecursive(
      ReactorInstance reactor, List<Event> events, Phase phase, TargetConfig targetConfig) {
    switch (phase) {
      case INIT_AND_PERIODIC:
        {
          // Add the startup trigger, if exists.
          var startup = reactor.getStartupTrigger();
          if (startup != null) events.add(new Event(startup, new Tag(0, 0, false)));

          // Add the initial timer firings, if exist.
          for (TimerInstance timer : reactor.timers) {
            events.add(new Event(timer, new Tag(timer.getOffset().toNanoSeconds(), 0, false)));
          }
          break;
        }
      case SHUTDOWN_TIMEOUT:
        {
          // To get the state space of the instant at shutdown,
          // we over-approximate by assuming all triggers are present at
          // (timeout, 0). This could generate unnecessary instructions
          // for reactions that are not meant to trigger at (timeout, 0),
          // but they will be treated as NOPs at runtime.

          // Add the shutdown trigger, if exists.
          var shutdown = reactor.getShutdownTrigger();
          if (shutdown != null) events.add(new Event(shutdown, new Tag(0, 0, false)));

          // Check for timers that fire at (timeout, 0).
          for (TimerInstance timer : reactor.timers) {
            // If timeout = timer.offset + N * timer.period for some non-negative
            // integer N, add a timer event.
            Long offset = timer.getOffset().toNanoSeconds();
            Long period = timer.getPeriod().toNanoSeconds();
            Long timeout = targetConfig.get(TimeOutProperty.INSTANCE).toNanoSeconds();
            if (period != 0 && (timeout - offset) % period == 0) {
              // The tag is set to (0,0) because, again, this is relative to the
              // shutdown phase, not the actual absolute tag at runtime.
              events.add(new Event(timer, new Tag(0, 0, false)));
            }
          }

          // Assume all input ports and logical actions present.
          // FIXME: Also physical action. Will add it later.
          for (PortInstance input : reactor.inputs) {
            events.add(new Event(input, new Tag(0, 0, false)));
          }
          for (ActionInstance logicalAction :
              reactor.actions.stream().filter(it -> !it.isPhysical()).toList()) {
            events.add(new Event(logicalAction, new Tag(0, 0, false)));
          }
          break;
        }
      case SHUTDOWN_STARVATION:
        {
          // Add the shutdown trigger, if exists.
          var shutdown = reactor.getShutdownTrigger();
          if (shutdown != null) events.add(new Event(shutdown, new Tag(0, 0, false)));
          break;
        }
      case ASYNC:
        {
          for (ActionInstance physicalAction :
              reactor.actions.stream().filter(it -> it.isPhysical()).toList()) {
            events.add(new Event(physicalAction, new Tag(0, 0, false)));
          }
          break;
        }
      default:
        throw new RuntimeException("UNREACHABLE");
    }

    // Recursion
    for (var child : reactor.children) {
      addInitialEventsRecursive(child, events, phase, targetConfig);
    }
  }

  /** Pop events with currentTag off an eventQ */
  private List<Event> popCurrentEvents(EventQueue eventQ, Tag currentTag) {
    List<Event> currentEvents = new ArrayList<>();
    // FIXME: Use stream methods here?
    while (eventQ.size() > 0 && eventQ.peek().getTag().compareTo(currentTag) == 0) {
      Event e = eventQ.poll();
      currentEvents.add(e);
    }
    return currentEvents;
  }

  /**
   * Return a list of reaction instances triggered by a list of current events. The events must
   * carry the same tag. Using a hash set here to make sure the reactions invoked are unique.
   * Sometimes multiple events can trigger the same reaction, and we do not want to record duplicate
   * reaction invocations.
   */
  private Set<ReactionInstance> getReactionsTriggeredByCurrentEvents(List<Event> currentEvents) {
    Set<ReactionInstance> reactions = new HashSet<>();
    for (Event e : currentEvents) {
      Set<ReactionInstance> dependentReactions = e.getTrigger().getDependentReactions();
      reactions.addAll(dependentReactions);
    }
    return reactions;
  }

  /**
   * Create a list of new events from reactions invoked at current tag. These new events should be
   * able to trigger reactions, which means that the method needs to compute how events propagate
   * downstream.
   *
   * <p>FIXME: This function does not handle port hierarchies, or the lack of them, yet. It should
   * be updated with a new implementation that uses eventualDestinations() from PortInstance.java.
   * But the challenge is to also get the delays. Perhaps eventualDestinations() should be extended
   * to collect delays.
   */
  private List<Event> createNewEvents(
      List<Event> currentEvents, Set<ReactionInstance> reactions, Tag currentTag) {

    List<Event> newEvents = new ArrayList<>();

    // If the event is a timer firing, enqueue the next firing.
    for (Event e : currentEvents) {
      if (e.getTrigger() instanceof TimerInstance) {
        TimerInstance timer = (TimerInstance) e.getTrigger();
        newEvents.add(
            new Event(
                timer,
                new Tag(
                    e.getTag().timestamp + timer.getPeriod().toNanoSeconds(),
                    0, // A time advancement resets microstep to 0.
                    false)));
      }
    }

    // For each reaction invoked, compute the new events produced
    // that can immediately trigger reactions.
    for (ReactionInstance reaction : reactions) {
      // Iterate over all the effects produced by this reaction.
      // If the effect is a port, obtain the downstream port along
      // a connection and enqueue a future event for that port.
      // If the effect is an action, enqueue a future event for
      // this action.
      for (TriggerInstance<? extends Variable> effect : reaction.effects) {
        // If the reaction writes to a port.
        if (effect instanceof PortInstance) {

          for (SendRange senderRange : ((PortInstance) effect).getDependentPorts()) {

            for (RuntimeRange<PortInstance> destinationRange : senderRange.destinations) {
              PortInstance downstreamPort = destinationRange.instance;

              // Getting delay from connection
              Expression delayExpr = senderRange.connection.getDelay();
              Long delay = ASTUtils.getDelay(delayExpr);
              if (delay == null) delay = 0L;

              // Create and enqueue a new event.
              Event e = new Event(downstreamPort, new Tag(currentTag.timestamp + delay, 0, false));
              newEvents.add(e);
            }
          }
        }
        // Ensure we only generate new events for LOGICAL actions.
        else if (effect instanceof ActionInstance && !((ActionInstance) effect).isPhysical()) {
          // Get the minimum delay of this action.
          long min_delay = ((ActionInstance) effect).getMinDelay().toNanoSeconds();
          long microstep = 0;
          if (min_delay == 0) {
            microstep = currentTag.microstep + 1;
          }
          // Create and enqueue a new event.
          Event e = new Event(effect, new Tag(currentTag.timestamp + min_delay, microstep, false));
          newEvents.add(e);
        }
      }
    }

    return newEvents;
  }
}
