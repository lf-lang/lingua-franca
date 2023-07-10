/**
 * Instance of a federate specification.
 *
 * <p>Copyright (c) 2020, The University of California at Berkeley.
 *
 * <p>Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * <p>1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and the following disclaimer.
 *
 * <p>2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * <p>THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * *************
 */
package org.lflang.federated.generator;

import com.google.common.base.Objects;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import org.lflang.MessageReporter;
import org.lflang.TargetConfig;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.ActionInstance;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.TriggerInstance;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.Expression;
import org.lflang.lf.Import;
import org.lflang.lf.ImportedReactor;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Output;
import org.lflang.lf.Parameter;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.Timer;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;

/**
 * Class that represents an instance of a federate, i.e., a reactor that is instantiated at the top
 * level of a federated reactor.
 *
 * @author Edward A. Lee
 * @author Soroush Bateni
 */
public class FederateInstance {

  /**
   * Construct a new federate instance on the basis of an instantiation in the federated reactor.
   *
   * @param instantiation The AST node of the instantiation.
   * @param id An identifier.
   * @param bankIndex If {@code instantiation.widthSpec !== null}, this gives the bank position.
   * @param messageReporter An object for reporting messages to the user.
   */
  public FederateInstance(
      Instantiation instantiation,
      int id,
      int bankIndex,
      TargetConfig targetConfig,
      MessageReporter messageReporter) {
    this.instantiation = instantiation;
    this.id = id;
    this.bankIndex = bankIndex;
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

  /** The host, if specified using the 'at' keyword. */
  public String host = "localhost";

  /** The instantiation of the top-level reactor, or null if there is no federation. */
  public Instantiation instantiation;

  public Instantiation getInstantiation() {
    return instantiation;
  }

  /** A list of individual connections between federates */
  public Set<FedConnectionInstance> connections = new HashSet<>();

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

  /**
   * A list of triggers for network input control reactions. This is used to trigger all the input
   * network control reactions that might be nested in a hierarchy.
   */
  public List<Action> networkInputControlReactionsTriggers = new ArrayList<>();

  /**
   * The trigger that triggers the output control reaction of this federate.
   *
   * <p>The network output control reactions send a PORT_ABSENT message for a network output port,
   * if it is absent at the current tag, to notify all downstream federates that no value will be
   * present on the given network port, allowing input control reactions on those federates to stop
   * blocking.
   */
  public Variable networkOutputControlReactionsTrigger = null;

  /** Indicates whether the federate is remote or local */
  public boolean isRemote = false;

  /**
   * List of generated network reactions (network receivers, network input control reactions,
   * network senders, and network output control reactions) that belong to this federate instance.
   */
  public List<Reaction> networkReactions = new ArrayList<>();

  /** Parsed target config of the federate. */
  public TargetConfig targetConfig;

  /** Keep a unique list of enabled serializers */
  public HashSet<SupportedSerializers> enabledSerializers = new HashSet<>();

  /** Cached result of analysis of which reactions to exclude from main. */
  private Set<Reaction> excludeReactions = null;

  /** An error reporter */
  private final MessageReporter messageReporter;

  /**
   * Return {@code true} if the class declaration of the given {@code instantiation} references the
   * {@code declaration} of a reactor class, either directly or indirectly (i.e, via a superclass or
   * a contained instantiation of the reactor class).
   *
   * @param declaration The reactor declaration to check whether it is referenced.
   */
  public boolean references(ReactorDecl declaration) {
    return references(this.instantiation, declaration);
  }

  /**
   * Return {@code true} if the class declaration of the given {@code instantiation} references the
   * {@code declaration} of a reactor class, either directly or indirectly (i.e, via a superclass or
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

    if (instantiation.getReactorClass() instanceof Reactor reactorDef) {
      // Check if the reactor is instantiated
      for (Instantiation child : reactorDef.getInstantiations()) {
        if (references(child, declaration)) {
          return true;
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
    return false;
  }

  /**
   * Return {@code true} if this federate references the given import.
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
   * Return {@code true} if this federate references the given parameter.
   *
   * @param param The parameter to check whether it is referenced.
   */
  public boolean references(Parameter param) {
    // Check if param is referenced in this federate's instantiation
    var returnValue =
        instantiation.getParameters().stream()
            .anyMatch(
                assignment ->
                    assignment.getRhs().getExprs().stream()
                        .filter(it -> it instanceof ParameterReference)
                        .map(it -> ((ParameterReference) it).getParameter())
                        .toList()
                        .contains(param));
    // If there are any user-defined top-level reactions, they could access
    // the parameters, so we need to include the parameter.
    var topLevelUserDefinedReactions =
        ((Reactor) instantiation.eContainer())
            .getReactions().stream()
                .filter(r -> !networkReactions.contains(r) && includes(r))
                .collect(Collectors.toCollection(ArrayList::new));
    returnValue |= !topLevelUserDefinedReactions.isEmpty();
    return returnValue;
  }

  /**
   * Return {@code true} if this federate includes a top-level reaction that references the given
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
            if (Objects.equal(triggerAsVarRef.getVariable(), action)) {
              return true;
            }
          }
        }
        // Look in sources
        for (VarRef source : convertToEmptyListIfNull(react.getSources())) {
          if (Objects.equal(source.getVariable(), action)) {
            return true;
          }
        }
        // Look in effects
        for (VarRef effect : convertToEmptyListIfNull(react.getEffects())) {
          if (Objects.equal(effect.getVariable(), action)) {
            return true;
          }
        }
      }
    }

    return false;
  }

  /**
   * Return {@code true} if the specified reaction should be included in the code generated for this
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

    if (networkReactions.contains(reaction)) {
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
   * Return {@code true} if this federate includes a top-level reaction that references the given
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
            if (Objects.equal(triggerAsVarRef.getVariable(), timer)) {
              return true;
            }
          }
        }
      }
    }
    return false;
  }

  /**
   * Return {@code true} if this federate instance includes the given instance.
   *
   * <p>NOTE: If the instance is a bank within the top level, then this returns {@code true} even
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
            .filter(it -> !networkReactions.contains(it))
            .collect(Collectors.toList());
    for (Reaction react : reactions) {
      // Create a collection of all the VarRefs (i.e., triggers, sources, and effects) in the react
      // signature that are ports that reference federates.
      // We then later check that all these VarRefs reference this federate. If not, we will add
      // this
      // react to the list of reactions that have to be excluded (note that mixing VarRefs from
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
      for (ReactionInstance reaction : output.getDependsOnReactions()) {
        TimeValue minDelay = findNearestPhysicalActionTrigger(reaction);
        if (!Objects.equal(minDelay, TimeValue.MAX_VALUE)) {
          physicalActionToOutputMinDelay.put((Output) output.getDefinition(), minDelay);
        }
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
   * Find the nearest (shortest) path to a physical action trigger from this 'reaction' in terms of
   * minimum delay.
   *
   * @param reaction The reaction to start with
   * @return The minimum delay found to the nearest physical action and TimeValue.MAX_VALUE
   *     otherwise
   */
  public TimeValue findNearestPhysicalActionTrigger(ReactionInstance reaction) {
    TimeValue minDelay = TimeValue.MAX_VALUE;
    for (TriggerInstance<? extends Variable> trigger : reaction.triggers) {
      if (trigger.getDefinition() instanceof Action action) {
        ActionInstance actionInstance = (ActionInstance) trigger;
        if (action.getOrigin() == ActionOrigin.PHYSICAL) {
          if (actionInstance.getMinDelay().isEarlierThan(minDelay)) {
            minDelay = actionInstance.getMinDelay();
          }
        } else if (action.getOrigin() == ActionOrigin.LOGICAL) {
          // Logical action
          // Follow it upstream inside the reactor
          for (ReactionInstance uReaction : actionInstance.getDependsOnReactions()) {
            // Avoid a loop
            if (!Objects.equal(uReaction, reaction)) {
              TimeValue uMinDelay =
                  actionInstance.getMinDelay().add(findNearestPhysicalActionTrigger(uReaction));
              if (uMinDelay.isEarlierThan(minDelay)) {
                minDelay = uMinDelay;
              }
            }
          }
        }

      } else if (trigger.getDefinition() instanceof Output) {
        // Outputs of contained reactions
        PortInstance outputInstance = (PortInstance) trigger;
        for (ReactionInstance uReaction : outputInstance.getDependsOnReactions()) {
          TimeValue uMinDelay = findNearestPhysicalActionTrigger(uReaction);
          if (uMinDelay.isEarlierThan(minDelay)) {
            minDelay = uMinDelay;
          }
        }
      }
    }
    return minDelay;
  }

  // TODO: Put this function into a utils file instead
  private <T> List<T> convertToEmptyListIfNull(List<T> list) {
    return list == null ? new ArrayList<>() : list;
  }
}
