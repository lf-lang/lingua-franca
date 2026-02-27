package org.lflang.federated.generator;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.SortedSet;
import java.util.TreeSet;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.lflang.MessageReporter;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.ActionInstance;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Action;
import org.lflang.lf.Connection;
import org.lflang.lf.Expression;
import org.lflang.lf.Import;
import org.lflang.lf.ImportedReactor;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Mode;
import org.lflang.lf.Output;
import org.lflang.lf.Parameter;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.Timer;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.VarRef;
import org.lflang.target.TargetConfig;

/**
 * Class that represents an instance of a federate, i.e., a reactor that is instantiated at the top
 * level of a federated reactor.
 *
 * @author Edward A. Lee
 * @author Soroush Bateni
 * @ingroup Federated
 */
public class FederateInstance {

  /**
   * Construct a new federate instance on the basis of an instantiation in the federated reactor.
   *
   * @param instantiation The AST node of the instantiation.
   * @param id An identifier.
   * @param bankIndex If `instantiation.widthSpec !== null`, this gives the bank position.
   * @param bankWidth The width of the bank.
   * @param targetConfig The target configuration.
   * @param messageReporter An object for reporting messages to the user.
   */
  public FederateInstance(
      Instantiation instantiation,
      int id,
      int bankIndex,
      int bankWidth,
      TargetConfig targetConfig,
      MessageReporter messageReporter) {
    this.instantiation = instantiation;
    this.id = id;
    this.bankIndex = bankIndex;
    this.bankWidth = bankWidth;
    this.messageReporter = messageReporter;
    this.targetConfig = targetConfig;

    // If the instantiation is in a bank, then we have to append
    // the bank index to the name.
    if (instantiation.getWidthSpec() != null) {
      this.name = "federate__" + instantiation.getName() + "__" + bankIndex;
    } else {
      this.name = "federate__" + instantiation.getName();
    }
  }

  /**
   * The position within a bank of reactors for this federate. This is 0 if the instantiation is not
   * a bank of reactors.
   */
  public int bankIndex;

  /**
   * The width of the bank in which this federate was instantiated. This is 1 if the instantiation
   * is not a bank of reactors.
   */
  public int bankWidth;

  /** The host, if specified using the 'at' keyword. */
  public String host = "localhost";

  /** The instantiation of the top-level reactor, or null if there is no federation. */
  public Instantiation instantiation;

  public Instantiation getInstantiation() {
    return instantiation;
  }

  /** A list of individual connections between federates */
  public Set<FedConnectionInstance> connections = new LinkedHashSet<>();

  /** The counter used to assign IDs to network senders. */
  public int networkIdSender = 0;

  /**
   * Map from the federates that this federate receives messages from to the delays on connections
   * from that federate. The delay set may include null, meaning that there is a connection from the
   * federate instance that has no delay.
   */
  public Map<FederateInstance, Set<Expression>> dependsOn = new LinkedHashMap<>();

  /** The port, if specified using the 'at' keyword. */
  public int port = 0;

  /**
   * Map from the federates that this federate sends messages to the delays on connections to that
   * federate. The delay set may include null, meaning that there is a connection from the federate
   * instance that has no delay.
   */
  public Map<FederateInstance, Set<Expression>> sendsTo = new LinkedHashMap<>();

  /** The user, if specified using the 'at' keyword. */
  public String user = null;

  /** The integer ID of this federate. */
  public int id;

  /**
   * The name of this federate instance. This will be the instantiation name, possibly appended with
   * "__n", where n is the bank position of this instance if the instantiation is of a bank of
   * reactors.
   */
  public String name;

  /**
   * List of networkMessage actions. Each of these handles a message received from another federate
   * over the network. The ID of receiving port is simply the position of the action in the list.
   * The sending federate needs to specify this ID.
   */
  public List<Action> networkMessageActions = new ArrayList<>();

  /** List of source federate IDs for networkMessage actions. */
  public List<FederateInstance> networkMessageSourceFederate = new ArrayList<>();

  /**
   * List of after delay values of the corresponding entries of `networkMessageActions`. These
   * will be `null` in the case of zero-delay connections and `0` in the case of
   * microstep-delay connections.
   */
  public List<Expression> networkMessageActionDelays = new ArrayList<>();

  /**
   * List of networkMessage actions corresponding to network input ports whose upstream federates
   * are in zero-delay cycles and the connection has no after delay. This should be a subset of the
   * networkMessageActions.
   */
  public List<Action> zeroDelayCycleNetworkMessageActions = new ArrayList<>();

  /**
   * A set of federates with which this federate has an inbound connection There will only be one
   * physical connection even if federate A has defined multiple physical connections to federate B.
   * The message handler on federate A will be responsible for including the appropriate information
   * in the message header (such as port ID) to help the receiver distinguish different events.
   */
  public Set<FederateInstance> inboundP2PConnections = new LinkedHashSet<>();

  /**
   * A list of federate with which this federate has an outbound physical connection. There will
   * only be one physical connection even if federate A has defined multiple physical connections to
   * federate B. The message handler on federate B will be responsible for distinguishing the
   * incoming messages by parsing their header and scheduling the appropriate action.
   */
  public Set<FederateInstance> outboundP2PConnections = new LinkedHashSet<>();

  /** Indicates whether the federate is remote or local */
  public boolean isRemote = false;

  /**
   * List of generated network reactions (network receivers) that belong to this federate instance.
   */
  public List<Reaction> networkReceiverReactions = new ArrayList<>();

  /** List of generated network reactions (network sender) that belong to this federate instance. */
  public List<Reaction> networkSenderReactions = new ArrayList<>();

  /**
   * List of generated network control reactions (network sender) that belong to this federate
   * instance.
   */
  public List<Reaction> portAbsentReactions = new ArrayList<>();

  /**
   * List of generated network reactors (network input and outputs) that belong to this federate
   * instance.
   */
  public List<Reactor> networkReactors = new ArrayList<>();

  /**
   * Mapping from a port instance of a connection to its associated network reaction. We populate
   * this map as we process connections as a means of annotating intra-federate dependencies
   */
  public Map<PortInstance, Instantiation> networkPortToInstantiation = new HashMap<>();

  /**
   * The mapping from network multiports of the federate to indexer reactors that split the
   * multiport into individual ports.
   */
  public Map<PortInstance, Instantiation> networkPortToIndexer = new HashMap<>();

  /**
   * List of generated network connections (network input and outputs) that belong to this federate
   * instance.
   */
  public List<Connection> networkConnections = new ArrayList<>();

  /**
   * List of generated network instantiations (network input and outputs) that belong to this
   * federate instance.
   */
  public List<Instantiation> networkSenderInstantiations = new ArrayList<>();

  /**
   * List of generated network instantiations (network input and outputs) that belong to this
   * federate instance.
   */
  public List<Instantiation> networkReceiverInstantiations = new ArrayList<>();

  /** List of generated instantiations that serve as helpers for forming the network connections. */
  public List<Instantiation> networkHelperInstantiations = new ArrayList<>();

  /** Parsed target config of the federate. */
  public TargetConfig targetConfig;

  /** Keep a unique list of enabled serializers */
  public HashSet<SupportedSerializers> enabledSerializers = new HashSet<>();

  /** Cached result of analysis of which reactions to exclude from main. */
  private Set<Reaction> excludeReactions = null;

  /** A list of unique STAA offsets over all input ports of this federate. */
  public SortedSet<TimeValue> staaOffsets = new TreeSet<TimeValue>();

  /** Keep a map of STP values to a list of network actions */
  public HashMap<TimeValue, List<Action>> staToNetworkActionMap = new HashMap<>();

  /** Keep a map of network actions to their associated instantiations */
  public HashMap<Action, Instantiation> networkActionToInstantiation = new HashMap<>();

  /** An message reporter */
  private final MessageReporter messageReporter;

  /**
   * Return `true` if the class declaration of the given `instantiation` references the
   * `declaration` of a reactor class, either directly or indirectly (i.e, via a superclass or
   * a contained instantiation of the reactor class).
   *
   * @param declaration The reactor declaration to check whether it is referenced.
   */
  public boolean references(ReactorDecl declaration) {
    return references(this.instantiation, declaration);
  }

  /**
   * Return `true` if the class declaration of the given `instantiation` references the
   * `declaration` of a reactor class, either directly or indirectly (i.e, via a superclass or
   * a contained instantiation of the reactor class).
   *
   * <p>An instantiation references the declaration of a reactor class if it is an instance of that
   * reactor class either directly or through inheritance, if its reactor class instantiates the
   * reactor class (or any contained instantiation does).
   *
   * @param instantiation The instantiation the class of which may refer to the reactor declaration.
   * @param declaration The reactor declaration to check whether it is referenced.
   */
  private boolean references(Instantiation instantiation, ReactorDecl declaration) {
    if (instantiation.getReactorClass().equals(ASTUtils.toDefinition(declaration))) {
      return true;
    }

    boolean instantiationsCheck = false;
    if (networkReactors.contains(ASTUtils.toDefinition(declaration))) {
      return true;
    }
    // For a federate, we don't need to look inside imported reactors.
    if (instantiation.getReactorClass() instanceof Reactor reactorDef) {
      // Check if the reactor is instantiated
      for (Instantiation child : reactorDef.getInstantiations()) {
        if (references(child, declaration)) {
          return true;
        }
      }
      // Check if it is instantiated in a mode
      for (Mode mode : reactorDef.getModes()) {
        for (Instantiation child : mode.getInstantiations()) {
          if (references(child, declaration)) {
            return true;
          }
        }
      }
      // Check if the reactor is a super class
      for (var parent : reactorDef.getSuperClasses()) {
        if (declaration instanceof Reactor r) {
          if (r.equals(parent)) {
            return true;
          }
          // Check if there are instantiations of the reactor in a super class
          if (parent instanceof Reactor p) {
            for (var inst : p.getInstantiations()) {
              if (references(inst, declaration)) {
                return true;
              }
            }
          }
        }
        if (declaration instanceof ImportedReactor i) {
          if (i.getReactorClass().equals(parent)) {
            return true;
          }
        }
      }
    }
    return instantiationsCheck;
  }

  /**
   * Return `true` if this federate references the given import.
   *
   * @param imp The import to check whether it is referenced.
   */
  public boolean references(Import imp) {
    for (ImportedReactor reactor : imp.getReactorClasses()) {
      if (this.references(reactor)) {
        return true;
      }
    }
    return false;
  }

  /**
   * Return `true` if this federate references the given parameter.
   *
   * @param param The parameter to check whether it is referenced.
   */
  public boolean references(Parameter param) {
    // Check if param is referenced in this federate's instantiation
    var returnValue =
        instantiation.getParameters().stream()
            .anyMatch(
                assignment -> {
                  final var expr = assignment.getRhs().getExpr();
                  if (expr instanceof ParameterReference) {
                    return ((ParameterReference) expr).getParameter().equals(param);
                  }
                  return false;
                });

    // If there are any user-defined top-level reactions, they could access
    // the parameters, so we need to include the parameter.
    var topLevelUserDefinedReactions =
        ((Reactor) instantiation.eContainer())
            .getReactions().stream()
                .filter(
                    r ->
                        !networkReceiverReactions.contains(r)
                            && !networkSenderReactions.contains(r)
                            && includes(r))
                .collect(Collectors.toCollection(ArrayList::new));
    returnValue |= !topLevelUserDefinedReactions.isEmpty();
    return returnValue;
  }

  /**
   * Return `true` if this federate includes a top-level reaction that references the given
   * action as a trigger, a source, or an effect.
   *
   * @param action The action to check whether it is to be included.
   */
  public boolean includes(Action action) {
    Reactor reactor = ASTUtils.getEnclosingReactor(action);
    // If the action is used as a trigger, a source, or an effect for a top-level reaction
    // that belongs to this federate, then generate it.
    for (Reaction react : ASTUtils.allReactions(reactor)) {
      if (includes(react)) {
        // Look in triggers
        for (TriggerRef trigger : convertToEmptyListIfNull(react.getTriggers())) {
          if (trigger instanceof VarRef triggerAsVarRef) {
            if (Objects.equals(triggerAsVarRef.getVariable(), action)) {
              return true;
            }
          }
        }
        // Look in sources
        for (VarRef source : convertToEmptyListIfNull(react.getSources())) {
          if (Objects.equals(source.getVariable(), action)) {
            return true;
          }
        }
        // Look in effects
        for (VarRef effect : convertToEmptyListIfNull(react.getEffects())) {
          if (Objects.equals(effect.getVariable(), action)) {
            return true;
          }
        }
      }
    }

    return false;
  }

  /**
   * Return `true` if the specified reaction should be included in the code generated for this
   * federate at the top-level. This means that if the reaction is triggered by or sends data to a
   * port of a contained reactor, then that reaction is in the federate. Otherwise, return false.
   *
   * <p>NOTE: This method assumes that it will not be called with reaction arguments that are within
   * other federates. It should only be called on reactions that are either at the top level or
   * within this federate. For this reason, for any reaction not at the top level, it returns true.
   *
   * @param reaction The reaction to check whether it is to be included.
   */
  public boolean includes(Reaction reaction) {
    Reactor reactor = ASTUtils.getEnclosingReactor(reaction);

    assert reactor != null;
    if (!reactor.getReactions().contains(reaction)) return false;

    if (networkReceiverReactions.contains(reaction) || networkSenderReactions.contains(reaction)) {
      // Reaction is a network reaction that belongs to this federate
      return true;
    }

    int reactionBankIndex = FedASTUtils.getReactionBankIndex(reaction);
    if (reactionBankIndex >= 0 && this.bankIndex >= 0 && reactionBankIndex != this.bankIndex) {
      return false;
    }

    // If this has been called before, then the result of the
    // following check is cached.
    if (excludeReactions != null) {
      return !excludeReactions.contains(reaction);
    }

    indexExcludedTopLevelReactions(reactor);

    return !excludeReactions.contains(reaction);
  }

  /**
   * Return `true` if this federate includes a top-level reaction that references the given
   * timer as a trigger, a source, or an effect.
   *
   * @param timer The action to check whether it is to be included.
   */
  public boolean includes(Timer timer) {
    Reactor reactor = ASTUtils.getEnclosingReactor(timer);

    for (Reaction r : ASTUtils.allReactions(reactor)) {
      if (includes(r)) {
        // Look in triggers
        for (TriggerRef trigger : convertToEmptyListIfNull(r.getTriggers())) {
          if (trigger instanceof VarRef triggerAsVarRef) {
            if (Objects.equals(triggerAsVarRef.getVariable(), timer)) {
              return true;
            }
          }
        }
      }
    }
    return false;
  }

  /**
   * Return `true` if this federate instance includes the given instance.
   *
   * <p>NOTE: If the instance is a bank within the top level, then this returns `true` even
   * though only one of the bank members is included in the federate.
   *
   * @param instance The reactor instance to check whether it is to be included.
   */
  public boolean includes(ReactorInstance instance) {
    // Start with this instance, then check its parents.
    ReactorInstance i = instance;
    while (i != null) {
      if (i.getDefinition() == instantiation) {
        return true;
      }
      i = i.getParent();
    }
    return false;
  }

  /**
   * Build an index of reactions at the top-level (in the federatedReactor) that don't belong to
   * this federate instance. This index is put in the excludeReactions class variable.
   *
   * @param federatedReactor The top-level federated reactor
   */
  private void indexExcludedTopLevelReactions(Reactor federatedReactor) {
    boolean inFederate;
    if (excludeReactions != null) {
      throw new IllegalStateException(
          "The index for excluded reactions at the top level is already built.");
    }

    excludeReactions = new LinkedHashSet<>();

    // Construct the set of excluded reactions for this federate.
    // If a reaction is a network reaction that belongs to this federate, we
    // don't need to perform this analysis.
    Iterable<Reaction> reactions =
        ASTUtils.allReactions(federatedReactor).stream()
            .filter(it -> !networkReceiverReactions.contains(it))
            .filter(it -> !networkSenderReactions.contains(it))
            .collect(Collectors.toList());
    for (Reaction react : reactions) {
      // Create a collection of all the VarRefs (i.e., triggers, sources, and effects) in the
      // reaction
      // signature that are ports that reference federates.
      // We then later check that all these VarRefs reference this federate. If not, we will add
      // this
      // reaction to the list of reactions that have to be excluded (note that mixing VarRefs from
      // different federates is not allowed).
      List<VarRef> allVarRefsReferencingFederates = new ArrayList<>();
      // Add all the triggers that are outputs
      Stream<VarRef> triggersAsVarRef =
          react.getTriggers().stream().filter(it -> it instanceof VarRef).map(it -> (VarRef) it);
      allVarRefsReferencingFederates.addAll(
          triggersAsVarRef.filter(it -> it.getVariable() instanceof Output).toList());
      // Add all the sources that are outputs
      allVarRefsReferencingFederates.addAll(
          react.getSources().stream().filter(it -> it.getVariable() instanceof Output).toList());
      // Add all the effects that are inputs
      allVarRefsReferencingFederates.addAll(
          react.getEffects().stream().filter(it -> it.getVariable() instanceof Input).toList());
      inFederate = containsAllVarRefs(allVarRefsReferencingFederates);
      if (!inFederate) {
        excludeReactions.add(react);
      }
    }
  }

  /** Cached result for isInZeroDelayCycle(). */
  private boolean _isInZeroDelayCycle = false;

  /**
   * Indicator that _isInZeroDelayCycle has been calculated. This will need to be reset if and when
   * mutations are supported.
   */
  private boolean _isInZeroDelayCycleCalculated = false;

  /**
   * Return true if there is a zero-delay path from this federate to itself. This is used to
   * determine whether absent messages need to be sent. Note that this is not the same as causality
   * loop detection. A federate may be in a zero-delay cycle (ZDC) even if there is no causality
   * loop.
   */
  public boolean isInZeroDelayCycle() {
    if (_isInZeroDelayCycleCalculated) return _isInZeroDelayCycle;
    _isInZeroDelayCycleCalculated = true;
    var visited = new HashSet<FederateInstance>();
    return _isInZeroDelayCycleInternal(this, this, visited);
  }

  /** Internal helper function for isInZeroDelayCycle(). */
  private boolean _isInZeroDelayCycleInternal(
      FederateInstance end, FederateInstance next, HashSet<FederateInstance> visited) {
    next.sendsTo.forEach(
        (destination, setOfDelays) -> {
          // Return if we've already found a cycle.
          // Also skip self loops because these get optimized away.
          // Also skip any we've visited.
          if (end._isInZeroDelayCycle
              || (end == next && destination == next)
              || visited.contains(destination)) return;
          if (setOfDelays.contains(null)) {
            // Only if we have a zero-delay connection to destination do we add it to visited.
            // If we have a delayed connection to destination, we should not skip a future
            // zero-delay
            // connection there.
            visited.add(destination);
            // There is a zero-delay connection to destination.
            if (destination == end) {
              // Found a zero delay cycle.
              end._isInZeroDelayCycle = true;
              return;
            }
            _isInZeroDelayCycleInternal(end, destination, visited);
          }
        });
    return end._isInZeroDelayCycle;
  }

  /**
   * Return true if all members of 'varRefs' belong to this federate.
   *
   * <p>As a convenience measure, if some members of 'varRefs' are from different federates, also
   * report an error.
   *
   * @param varRefs A collection of VarRefs
   */
  private boolean containsAllVarRefs(Iterable<VarRef> varRefs) {
    var referencesFederate = false;
    var inFederate = true;
    for (VarRef varRef : varRefs) {
      if (varRef.getContainer() == this.instantiation) {
        referencesFederate = true;
      } else {
        if (referencesFederate) {
          messageReporter
              .at(varRef)
              .error("Mixed triggers and effects from different federates. This is not permitted");
        }
        inFederate = false;
      }
    }
    return inFederate;
  }

  /**
   * Return the first found physical action or null if there is no physical action in this federate.
   *
   * @param instance The reactor instance to check whether there is a physical action.
   */
  public ActionInstance findPhysicalAction(ReactorInstance instance) {
    for (ActionInstance action : instance.actions) {
      if (action.isPhysical()) {
        return action;
      }
    }
    for (ReactorInstance child : instance.children) {
      for (ActionInstance action : child.actions) {
        if (action.isPhysical()) {
          return action;
        }
      }
    }
    return null;
  }

  /**
   * Find output ports that are connected to a physical action trigger upstream in the same reactor.
   * Return a list of such outputs paired with the minimum delay from the nearest physical action.
   *
   * @param instance The reactor instance containing the output ports
   * @return A LinkedHashMap<Output, TimeValue>
   */
  public LinkedHashMap<Output, TimeValue> findOutputsConnectedToPhysicalActions(
      ReactorInstance instance) {
    LinkedHashMap<Output, TimeValue> physicalActionToOutputMinDelay = new LinkedHashMap<>();
    // Find reactions that write to the output port of the reactor
    for (PortInstance output : instance.outputs) {
      TimeValue minDelay = minDelayFromPhysicalActionTo(null, output);
      if (!Objects.equals(minDelay, TimeValue.MAX_VALUE)) {
        physicalActionToOutputMinDelay.put((Output) output.getDefinition(), minDelay);
      }
    }
    return physicalActionToOutputMinDelay;
  }

  @Override
  public String toString() {
    return "Federate "
        + id
        + ": "
        + ((instantiation != null) ? instantiation.getName() : "no name");
  }

  /**
   * Return the shortest total delay from an upstream physical action to the specified output port
   * or TimeValue.MAX_VALUE if there is no upstream physical action.
   *
   * @param port A port (either input or output) wrt which the min delay is a calculated.
   */
  private TimeValue minDelayFromPhysicalActionTo(
      Map<ReactionInstance, TimeValue> visited, PortInstance port) {
    if (visited == null) {
      visited = new HashMap<ReactionInstance, TimeValue>();
    }
    // For each output port, it may depend on reactions or be connected to upstream ports or both.
    TimeValue result = TimeValue.MAX_VALUE;
    // Check reactions that write directly to this port first.
    for (ReactionInstance reaction : port.getDependsOnReactions()) {
      TimeValue minDelay = minDelayFromPhysicalActionTo(visited, reaction);
      if (minDelay.isEarlierThan(result)) result = minDelay;
    }
    // Check upstream ports that connect to this port.
    for (var upstreamPort : port.getDependsOnPorts()) {
      var minDelayOnConnections = port.minDelayFrom(upstreamPort.instance);
      for (var reaction : upstreamPort.instance.getDependsOnReactions()) {
        var minDelayToReaction =
            minDelayFromPhysicalActionTo(visited, reaction).add(minDelayOnConnections);
        if (minDelayToReaction.isEarlierThan(result)) result = minDelayToReaction;
      }
    }
    return result;
  }

  /**
   * If the specified time is less than the time already stored in the map for the specified
   * reaction, or if there is no time stored in the map, then set the map to the specified time.
   *
   * @param map The map.
   * @param time The time.
   * @return True if the value was replaced.
   */
  private boolean replaceIfLess(
      Map<ReactionInstance, TimeValue> map, ReactionInstance reaction, TimeValue time) {
    var previous = map.get(reaction);
    if (previous == null || time.isEarlierThan(previous)) {
      map.put(reaction, time);
      return true;
    }
    return false;
  }

  /**
   * Return the shortest total delay from an upstream physical action to the specified reaction.
   *
   * @param visited A set of reactions that have been visited used to avoid deep loops.
   * @param reaction The reaction.
   * @return The minimum delay found to the nearest physical action and TimeValue.MAX_VALUE if there
   *     is no upstream physical action.
   */
  private TimeValue minDelayFromPhysicalActionTo(
      Map<ReactionInstance, TimeValue> visited, ReactionInstance reaction) {
    if (visited == null) {
      visited = new HashMap<ReactionInstance, TimeValue>();
    }
    var previousDelay = visited.get(reaction);
    if (previousDelay != null) {
      // The reaction is either in progress or resolved. Either way, return its delay.
      return previousDelay;
    }
    visited.put(reaction, TimeValue.MAX_VALUE);
    for (var trigger : reaction.triggers) {
      if (trigger instanceof ActionInstance action) {
        var actionDelay = action.getMinDelay();
        if (action.isPhysical()) {
          replaceIfLess(visited, reaction, actionDelay);
        } else {
          // Logical action. Follow it upstream.
          // Assume that all after delays have been converted to delay reactors, which use logical
          // actions.
          for (ReactionInstance uReaction : action.getDependsOnReactions()) {
            var uDelay = minDelayFromPhysicalActionTo(visited, uReaction).add(actionDelay);
            replaceIfLess(visited, reaction, uDelay);
          }
        }
      } else if (trigger instanceof PortInstance port) {
        // Regardless of whether the port is an output of a contained reactor or an input of
        // the container reactor, recurse on reactions that write to it as well as the upstream
        // ports connected to it.
        for (ReactionInstance uReaction : port.getDependsOnReactions()) {
          var uDelay = minDelayFromPhysicalActionTo(visited, uReaction);
          replaceIfLess(visited, reaction, uDelay);
        }
        for (var upstreamPort : port.getDependsOnPorts()) {
          var uDelay = minDelayFromPhysicalActionTo(visited, upstreamPort.instance);
          // Add the connection delay.
          var connectionDelay = port.minDelayFrom(upstreamPort.instance);
          replaceIfLess(visited, reaction, uDelay.add(connectionDelay));
        }
      }
    }
    return visited.get(reaction);
  }

  // TODO: Put this function into a utils file instead
  private <T> List<T> convertToEmptyListIfNull(List<T> list) {
    return list == null ? new ArrayList<>() : list;
  }
}
