/*************
 * Copyright (c) 2021, The University of California at Berkeley.
 * Copyright (c) 2021, The University of Texas at Dallas.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.federated.generator;

import com.google.common.collect.Iterators;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.stream.Collectors;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.lflang.InferredType;
import org.lflang.MessageReporter;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.federated.extensions.FedTargetExtension;
import org.lflang.federated.extensions.FedTargetExtensionFactory;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.MixedRadixInt;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.BuiltinTrigger;
import org.lflang.lf.BuiltinTriggerRef;
import org.lflang.lf.Connection;
import org.lflang.lf.Expression;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Model;
import org.lflang.lf.Output;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.Type;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;

/**
 * A helper class for AST transformations needed for federated execution.
 *
 * @author Soroush Bateni
 * @author Edward A. Lee
 */
public class FedASTUtils {

  /** Map from reactions to bank indices */
  private static Map<Reaction, Integer> reactionBankIndices = null;

  /**
   * Mark the specified reaction to belong to only the specified bank index. This is needed because
   * reactions cannot declare a specific bank index as an effect or trigger. Reactions that send
   * messages between federates, including absent messages, need to be specific to a bank member.
   *
   * @param reaction The reaction.
   * @param bankIndex The bank index, or -1 if there is no bank.
   */
  public static void setReactionBankIndex(Reaction reaction, int bankIndex) {
    if (bankIndex < 0) {
      return;
    }
    if (reactionBankIndices == null) {
      reactionBankIndices = new LinkedHashMap<>();
    }
    reactionBankIndices.put(reaction, bankIndex);
  }

  /**
   * Return the reaction bank index.
   *
   * @see #setReactionBankIndex(Reaction reaction, int bankIndex)
   * @param reaction The reaction.
   * @return The reaction bank index, if one has been set, and -1 otherwise.
   */
  public static int getReactionBankIndex(Reaction reaction) {
    if (reactionBankIndices == null) return -1;
    if (reactionBankIndices.get(reaction) == null) return -1;
    return reactionBankIndices.get(reaction);
  }

  /**
   * Find the federated reactor in a .lf file.
   *
   * @param resource Resource representing a .lf file.
   * @return The federated reactor if found.
   */
  public static Reactor findFederatedReactor(Resource resource) {
    return IteratorExtensions.findFirst(
        Iterators.filter(resource.getAllContents(), Reactor.class), Reactor::isFederated);
  }

  /**
   * Replace the specified connection with communication between federates.
   *
   * @param connection Network connection between two federates.
   * @param resource The resource from which the ECore model was derived.
   * @param coordination One of CoordinationType.DECENTRALIZED or CoordinationType.CENTRALIZED.
   * @param messageReporter Used to report errors encountered.
   */
  public static void makeCommunication(
      FedConnectionInstance connection,
      Resource resource,
      CoordinationType coordination,
      MessageReporter messageReporter) {

    addNetworkSenderReactor(connection, coordination, resource, messageReporter);

    FedASTUtils.addPortAbsentReaction(connection);

    addNetworkReceiverReactor(connection, coordination, resource, messageReporter);
  }

  /**
   * Create a "network action" in the reactor that contains the given connection and return it.
   *
   * <p>The purpose of this action is to serve as a trigger for a "network input reaction" that is
   * responsible for relaying messages to the port that is on the receiving side of the given
   * connection. The connection is assumed to be between two reactors that reside in distinct
   * federates. Hence, the container of the connection is assumed to be top-level.
   *
   * @param connection A connection between two federates
   * @return The newly created action.
   */
  private static Action createNetworkAction(FedConnectionInstance connection) {
    // Reactor top = (Reactor) connection.getDefinition().eContainer();
    LfFactory factory = LfFactory.eINSTANCE;

    Action action = factory.createAction();
    // Name the newly created action; set its delay and type.
    action.setName("networkMessage");
    if (connection.serializer == SupportedSerializers.NATIVE) {
      action.setType(EcoreUtil.copy(connection.getSourcePortInstance().getDefinition().getType()));
    } else {
      Type action_type = factory.createType();
      action_type.setId(
          FedTargetExtensionFactory.getExtension(connection.srcFederate.targetConfig.target)
              .getNetworkBufferType());
      action.setType(action_type);
    }

    // The connection is 'physical' if it uses the ~> notation.
    if (connection.getDefinition().isPhysical()) {
      action.setOrigin(ActionOrigin.PHYSICAL);
      // Messages sent on physical connections do not
      // carry a timestamp, or a delay. The delay
      // provided using after is enforced by setting
      // the minDelay.
      if (connection.getDefinition().getDelay() != null) {
        action.setMinDelay(connection.getDefinition().getDelay());
      }
    } else {
      action.setOrigin(ActionOrigin.LOGICAL);
    }

    return action;
  }

  /** Add a reactor definition with the given name to the given resource and return it. */
  public static Reactor addReactorDefinition(String name, Resource resource) {
    var reactor = LfFactory.eINSTANCE.createReactor();
    reactor.setName(name);
    EObject node = IteratorExtensions.findFirst(resource.getAllContents(), Model.class::isInstance);
    ((Model) node).getReactors().add(reactor);
    return reactor;
  }

  /**
   * Add a network receiver reactor for a given input port 'destination' to destination's parent
   * reactor. This reaction will react to a generated 'networkAction' (triggered asynchronously,
   * e.g., by federate.c). This 'networkAction' will contain the actual message that is sent by the
   * sender in 'action->value'. This value is forwarded to 'destination' in the network receiver
   * reaction.
   *
   * @param connection A description of the federated connection that is being replaced.
   * @param coordination One of CoordinationType.DECENTRALIZED or CoordinationType.CENTRALIZED.
   * @param resource The resource from which the ECore model was derived.
   */
  private static void addNetworkReceiverReactor(
      FedConnectionInstance connection,
      CoordinationType coordination,
      Resource resource,
      MessageReporter messageReporter) {
    LfFactory factory = LfFactory.eINSTANCE;
    var extension =
        FedTargetExtensionFactory.getExtension(connection.srcFederate.targetConfig.target);
    Type type = EcoreUtil.copy(connection.getDestinationPortInstance().getDefinition().getType());

    VarRef sourceRef = factory.createVarRef(); // source fed
    VarRef instRef = factory.createVarRef(); // instantiation connection
    VarRef destRef = factory.createVarRef(); // destination connection

    Reactor receiver = addReactorDefinition("NetworkReceiver_" + networkIDReceiver++, resource);
    Reaction networkReceiverReaction = factory.createReaction();

    Output out = factory.createOutput();
    VarRef outRef = factory.createVarRef(); // out port
    Connection receiverFromReaction = factory.createConnection();
    Instantiation networkInstance = factory.createInstantiation();

    Reactor top =
        connection
            .getSourcePortInstance()
            .getParent()
            .getParent()
            .reactorDefinition; // Top-level reactor.

    // Add the attribute "_NetworkReactor" for the network receiver.
    var a = factory.createAttribute();
    a.setAttrName("_NetworkReactor");
    var e = factory.createAttrParm();
    e.setValue("Receiver");
    a.getAttrParms().add(e);
    receiver.getAttributes().add(a);

    receiver
        .getReactions()
        .add(getInitializationReaction(extension, extension.inputInitializationBody()));

    receiver.getReactions().add(networkReceiverReaction);
    receiver.getOutputs().add(out);

    addLevelAttribute(
        networkInstance,
        connection.getDestinationPortInstance(),
        getDstIndex(connection),
        connection);
    networkInstance.setReactorClass(receiver);
    networkInstance.setName(
        ASTUtils.getUniqueIdentifier(top, "nr_" + connection.getDstFederate().name));
    top.getInstantiations().add(networkInstance);

    receiverFromReaction.getLeftPorts().add(instRef);
    receiverFromReaction.getRightPorts().add(destRef);

    // Create the network action (@see createNetworkAction)
    Action networkAction = createNetworkAction(connection);

    // Keep track of this action in the destination federate.
    connection.dstFederate.networkMessageActions.add(networkAction);
    if (connection.getDefinition().getDelay() == null)
      connection.dstFederate.zeroDelayNetworkMessageActions.add(networkAction);

    TimeValue maxSTP = findMaxSTP(connection, coordination);

    if (!connection.dstFederate.currentSTPOffsets.contains(maxSTP.time)) {
      connection.dstFederate.currentSTPOffsets.add(maxSTP.time);
      connection.dstFederate.stpOffsets.add(maxSTP);
      connection.dstFederate.stpToNetworkActionMap.put(maxSTP, new ArrayList<>());
    } else {
      // TODO: Find more efficient way to reuse timevalues
      for (var offset : connection.dstFederate.stpOffsets) {
        if (maxSTP.time == offset.time) {
          maxSTP = offset;
          break;
        }
      }
    }

    connection.dstFederate.stpToNetworkActionMap.get(maxSTP).add(networkAction);

    // Add the action definition to the parent reactor.
    receiver.getActions().add(networkAction);

    // If the sender or receiver is in a bank of reactors, then we want
    // these reactions to appear only in the federate whose bank ID matches.
    setReactionBankIndex(networkReceiverReaction, connection.getDstBank());

    // FIXME: do not create a new extension every time it is used
    extension.annotateReaction(networkReceiverReaction);

    // The connection is 'physical' if it uses the ~> notation.
    if (connection.getDefinition().isPhysical()) {
      connection.dstFederate.inboundP2PConnections.add(connection.srcFederate);
    } else {
      // If the connection is logical but coordination
      // is decentralized, we would need
      // to make P2P connections
      if (coordination == CoordinationType.DECENTRALIZED) {
        connection.dstFederate.inboundP2PConnections.add(connection.srcFederate);
      }
    }

    // Establish references to the involved ports.
    sourceRef.setContainer(connection.getSourcePortInstance().getParent().getDefinition());
    sourceRef.setVariable(connection.getSourcePortInstance().getDefinition());
    destRef.setContainer(
        connection
            .getDstFederate()
            .networkPortToIndexer
            .get(connection.getDestinationPortInstance()));
    var v = LfFactory.eINSTANCE.createVariable();
    v.setName("port" + connection.getDstChannel());
    destRef.setVariable(v);
    instRef.setContainer(networkInstance);
    instRef.setVariable(out);

    out.setName("msg");
    out.setType(type);
    outRef.setVariable(out);

    VarRef triggerRef = factory.createVarRef();
    // Establish references to the action.
    triggerRef.setVariable(networkAction);
    // Add the action as a trigger to the receiver reaction
    networkReceiverReaction.getTriggers().add(triggerRef);
    networkReceiverReaction.getEffects().add(outRef);

    // Generate code for the network receiver reaction
    networkReceiverReaction.setCode(factory.createCode());
    networkReceiverReaction
        .getCode()
        .setBody(
            FedTargetExtensionFactory.getExtension(connection.dstFederate.targetConfig.target)
                .generateNetworkReceiverBody(
                    networkAction,
                    sourceRef,
                    outRef,
                    connection,
                    ASTUtils.getInferredType(networkAction),
                    coordination,
                    messageReporter));

    // Add the network receiver reaction to the federate instance's list
    // of network reactions
    connection.dstFederate.networkReceiverReactions.add(networkReceiverReaction);
    connection.dstFederate.networkReactors.add(receiver);
    connection.dstFederate.networkConnections.add(receiverFromReaction);
    connection.dstFederate.networkReceiverInstantiations.add(networkInstance);
    connection.dstFederate.networkPortToInstantiation.put(
        connection.getDestinationPortInstance(), networkInstance);
    connection.dstFederate.networkActionToInstantiation.put(networkAction, networkInstance);
  }

  private static MixedRadixInt getSrcIndex(FedConnectionInstance connection) {
    var widths = List.of(connection.srcRange.instance.getWidth(), connection.srcFederate.bankWidth);
    var digits = List.of(connection.getSrcChannel(), connection.getSrcBank());
    return new MixedRadixInt(digits, widths, List.of(0, 1));
  }

  private static MixedRadixInt getDstIndex(FedConnectionInstance connection) {
    var widths = List.of(connection.dstRange.instance.getWidth(), connection.dstFederate.bankWidth);
    var digits = List.of(connection.getDstChannel(), connection.getDstBank());
    return new MixedRadixInt(digits, widths, List.of(0, 1));
  }

  /** Add a level annotation to the instantiation of a network reactor. */
  private static void addLevelAttribute(
      Instantiation instantiation,
      PortInstance p,
      MixedRadixInt index,
      FedConnectionInstance connection) {
    if (connection.getDefinition().getDelay() != null) return;
    var a = LfFactory.eINSTANCE.createAttribute();
    a.setAttrName("_tpoLevel");
    var e = LfFactory.eINSTANCE.createAttrParm();
    // preserve relative orderings according to the downstream reaction, but also ensure that the
    // output and the input
    // that it is connected to, which both have the same downstream reaction, have the correct
    // ordering wrt each other.
    var ub = p.getLevelUpperBound(index);
    e.setValue(String.valueOf(p.isInput() ? 2 * ub : 2 * ub - 1));
    a.getAttrParms().add(e);
    instantiation.getAttributes().add(a);
  }

  /**
   * Go upstream from input port {@code port} until we reach one or more output ports that belong to
   * the same federate.
   *
   * <p>Along the path, we follow direct connections, as well as reactions, as long as there is no
   * logical delay. When following reactions, we also follow dependant reactions (because we are
   * traversing a potential cycle backwards).
   *
   * @return A set of {@link PortInstance}. If no port exist that match the criteria, return an
   *     empty set.
   */
  private static Set<PortInstance> findUpstreamPortsInFederate(
      FederateInstance federate,
      PortInstance port,
      Set<PortInstance> visitedPorts,
      Set<ReactionInstance> reactionVisited) {
    Set<PortInstance> toReturn = new HashSet<>();
    if (port == null) return toReturn;
    else if (ASTUtils.isTopLevel(port.getParent()) || federate.includes(port.getParent())) {
      // Reached the requested federate
      toReturn.add(port);
      visitedPorts.add(port);
    } else if (visitedPorts.contains(port)) {
      return toReturn;
    } else {
      visitedPorts.add(port);
      // Follow depends on reactions
      port.getDependsOnReactions()
          .forEach(
              reaction ->
                  followReactionUpstream(
                      federate, visitedPorts, toReturn, reaction, reactionVisited));
      // Follow depends on ports
      port.getDependsOnPorts()
          .forEach(
              it ->
                  toReturn.addAll(
                      findUpstreamPortsInFederate(
                          federate, it.instance, visitedPorts, reactionVisited)));
    }
    return toReturn;
  }

  /**
   * Follow reactions upstream. This is part of the algorithm of {@link
   * #findUpstreamPortsInFederate}.
   */
  private static void followReactionUpstream(
      FederateInstance federate,
      Set<PortInstance> visitedPorts,
      Set<PortInstance> toReturn,
      ReactionInstance reaction,
      Set<ReactionInstance> reactionVisited) {
    if (reactionVisited.contains(reaction)) return;
    reactionVisited.add(reaction);
    // Add triggers
    Set<VarRef> varRefsToFollow = new HashSet<>();
    varRefsToFollow.addAll(
        reaction.getDefinition().getTriggers().stream()
            .filter(trigger -> !(trigger instanceof BuiltinTriggerRef))
            .map(VarRef.class::cast)
            .toList());
    // Add sources
    varRefsToFollow.addAll(reaction.getDefinition().getSources());

    // Follow everything upstream
    varRefsToFollow.forEach(
        varRef ->
            toReturn.addAll(
                findUpstreamPortsInFederate(
                    federate,
                    reaction.getParent().lookupPortInstance(varRef),
                    visitedPorts,
                    reactionVisited)));

    reaction.dependsOnReactions().stream()
        .filter(
            // Stay within the reactor
            it -> it.getParent().equals(reaction.getParent()))
        .forEach(
            it -> followReactionUpstream(federate, visitedPorts, toReturn, it, reactionVisited));

    // FIXME: This is most certainly wrong. Please fix it.
    reaction.dependentReactions().stream()
        .filter(
            // Stay within the reactor
            it -> it.getParent().equals(reaction.getParent()))
        .forEach(
            it -> followReactionUpstream(federate, visitedPorts, toReturn, it, reactionVisited));
  }

  /**
   * Find the maximum STP offset for the given 'port'.
   *
   * <p>An STP offset predicate can be nested in contained reactors in the federate.
   *
   * @param connection The connection to find the max STP offset for.
   * @param coordination The coordination scheme.
   * @return The maximum STP as a TimeValue
   */
  private static TimeValue findMaxSTP(
      FedConnectionInstance connection, CoordinationType coordination) {
    Variable port = connection.getDestinationPortInstance().getDefinition();
    FederateInstance instance = connection.dstFederate;
    Reactor reactor = connection.getDestinationPortInstance().getParent().reactorDefinition;

    // Find a list of STP offsets (if any exists)
    List<Expression> STPList = new LinkedList<>();

    // First, check if there are any connections to contained reactors that
    // need to be handled
    List<Connection> connectionsWithPort =
        ASTUtils.allConnections(reactor).stream()
            .filter(
                c -> c.getLeftPorts().stream().anyMatch((VarRef v) -> v.getVariable().equals(port)))
            .collect(Collectors.toList());

    // Find the list of reactions that have the port as trigger or source
    // (could be a variable name)
    List<Reaction> reactionsWithPort =
        ASTUtils.allReactions(reactor).stream()
            .filter(
                r -> {
                  // Check the triggers of reaction r first
                  return r.getTriggers().stream()
                          .anyMatch(
                              t -> {
                                if (t instanceof VarRef) {
                                  // Check if the variables match
                                  return ((VarRef) t).getVariable() == port;
                                } else {
                                  // Not a network port (startup or shutdown)
                                  return false;
                                }
                              })
                      || // Then check the sources of reaction r
                      r.getSources().stream().anyMatch(s -> s.getVariable() == port);
                })
            .collect(Collectors.toList());

    // Find a list of STP offsets (if any exists)
    if (coordination == CoordinationType.DECENTRALIZED) {
      for (Reaction r : safe(reactionsWithPort)) {
        // If STP offset is determined, add it
        // If not, assume it is zero
        if (r.getStp() != null) {
          if (r.getStp().getValue() instanceof ParameterReference) {
            List<Instantiation> instantList = new ArrayList<>();
            instantList.add(instance.instantiation);
            final var param = ((ParameterReference) r.getStp().getValue()).getParameter();
            STPList.addAll(ASTUtils.initialValue(param, instantList));
          } else {
            STPList.add(r.getStp().getValue());
          }
        }
      }
      // Check the children for STPs as well
      for (Connection c : safe(connectionsWithPort)) {
        VarRef childPort = c.getRightPorts().get(0);
        Reactor childReactor = (Reactor) childPort.getVariable().eContainer();
        // Find the list of reactions that have the port as trigger or
        // source (could be a variable name)
        List<Reaction> childReactionsWithPort =
            ASTUtils.allReactions(childReactor).stream()
                .filter(
                    r ->
                        r.getTriggers().stream()
                                .anyMatch(
                                    t -> {
                                      if (t instanceof VarRef) {
                                        // Check if the variables match
                                        return ((VarRef) t).getVariable()
                                            == childPort.getVariable();
                                      } else {
                                        // Not a network port (startup or shutdown)
                                        return false;
                                      }
                                    })
                            || r.getSources().stream()
                                .anyMatch(s -> s.getVariable() == childPort.getVariable()))
                .collect(Collectors.toList());

        for (Reaction r : safe(childReactionsWithPort)) {
          // If STP offset is determined, add it
          // If not, assume it is zero
          if (r.getStp() != null) {
            if (r.getStp().getValue() instanceof ParameterReference) {
              List<Instantiation> instantList = new ArrayList<>();
              instantList.add(childPort.getContainer());
              final var param = ((ParameterReference) r.getStp().getValue()).getParameter();
              STPList.addAll(ASTUtils.initialValue(param, instantList));
            } else {
              STPList.add(r.getStp().getValue());
            }
          }
        }
      }
    }

    return STPList.stream()
        .map(ASTUtils::getLiteralTimeValue)
        .filter(Objects::nonNull)
        .reduce(TimeValue.ZERO, TimeValue::max);
  }

  /**
   * Return a null-safe List
   *
   * @param <E> The type of the list
   * @param list The potentially null List
   * @return Empty list or the original list
   */
  public static <E> List<E> safe(List<E> list) {
    return list == null ? Collections.emptyList() : list;
  }

  public static int networkIDReceiver = 0;

  /** The network sender reactors created for each connection. */
  private static final Map<FedConnectionInstance, Reactor> networkSenderReactors =
      new HashMap<>(); // FIXME: static mutable objects are bad

  /**
   * Generate and return the EObject representing the reactor definition of a network sender.
   *
   * @param connection The connection the communication over which is handled by an instance of the
   *     reactor to be returned.
   * @param coordination Centralized or decentralized.
   * @param resource The resource of the ECore model to which the new reactor definition should be
   *     added.
   */
  private static Reactor getNetworkSenderReactor(
      FedConnectionInstance connection,
      CoordinationType coordination,
      Resource resource,
      MessageReporter messageReporter) {
    var extension =
        FedTargetExtensionFactory.getExtension(connection.srcFederate.targetConfig.target);
    LfFactory factory = LfFactory.eINSTANCE;
    Type type = EcoreUtil.copy(connection.getSourcePortInstance().getDefinition().getType());

    VarRef inRef = factory.createVarRef(); // in port to network reaction
    VarRef destRef = factory.createVarRef(); // destination fed

    // Initialize Reactor and Reaction AST Nodes
    Reactor sender = factory.createReactor();

    // Add the attribute "_NetworkReactor" for the network sender.
    var a = factory.createAttribute();
    a.setAttrName("_NetworkReactor");
    var e = factory.createAttrParm();
    e.setValue("Sender");
    a.getAttrParms().add(e);
    sender.getAttributes().add(a);

    Input in = factory.createInput();
    in.setName("msg");
    in.setType(type);
    in.setWidthSpec(
        EcoreUtil.copy(connection.getSourcePortInstance().getDefinition().getWidthSpec()));
    inRef.setVariable(in);

    destRef.setContainer(connection.getDestinationPortInstance().getParent().getDefinition());
    destRef.setVariable(connection.getDestinationPortInstance().getDefinition());

    Reaction networkSenderReaction =
        getNetworkSenderReaction(inRef, destRef, connection, coordination, type, messageReporter);

    extension.addSenderIndexParameter(sender);

    sender
        .getReactions()
        .add(getInitializationReaction(extension, extension.outputInitializationBody()));
    sender.getReactions().add(networkSenderReaction);
    sender.getInputs().add(in);

    EObject node = IteratorExtensions.findFirst(resource.getAllContents(), Model.class::isInstance);
    ((Model) node).getReactors().add(sender);
    sender.setName("NetworkSender_" + connection.getSrcFederate().networkIdSender);

    // FIXME: do not create a new extension every time it is used
    extension.annotateReaction(networkSenderReaction);

    // If the sender or receiver is in a bank of reactors, then we want
    // these reactions to appear only in the federate whose bank ID matches.
    setReactionBankIndex(networkSenderReaction, connection.getSrcBank());

    // Add the network sender reaction to the federate instance's list
    // of network reactions
    connection.srcFederate.networkSenderReactions.add(networkSenderReaction);
    connection.srcFederate.networkReactors.add(sender);

    networkSenderReactors.put(connection, sender);

    return sender;
  }

  /** Return the reaction that sends messages when its corresponding port is present. */
  private static Reaction getNetworkSenderReaction(
      VarRef inRef,
      VarRef destRef,
      FedConnectionInstance connection,
      CoordinationType coordination,
      Type type,
      MessageReporter messageReporter) {
    var networkSenderReaction = LfFactory.eINSTANCE.createReaction();
    networkSenderReaction.getTriggers().add(inRef);
    networkSenderReaction.setCode(LfFactory.eINSTANCE.createCode());
    networkSenderReaction
        .getCode()
        .setBody(
            FedTargetExtensionFactory.getExtension(connection.srcFederate.targetConfig.target)
                .generateNetworkSenderBody(
                    inRef,
                    destRef,
                    connection,
                    InferredType.fromAST(type),
                    coordination,
                    messageReporter));
    return networkSenderReaction;
  }

  /**
   * Return the reaction that initializes the containing network sender reactor on {@code startup}.
   */
  private static Reaction getInitializationReaction(FedTargetExtension extension, String body) {
    var initializationReaction = LfFactory.eINSTANCE.createReaction();
    var startup = LfFactory.eINSTANCE.createBuiltinTriggerRef();
    startup.setType(BuiltinTrigger.STARTUP);
    extension.annotateReaction(initializationReaction);
    initializationReaction.getTriggers().add(startup);
    var code = LfFactory.eINSTANCE.createCode();
    code.setBody(body);
    initializationReaction.setCode(code);
    return initializationReaction;
  }

  /**
   * Add a network sender reactor for a given input port 'source' to source's parent reactor. This
   * reaction will react to the 'source' and then send a message on the network destined for the
   * destinationFederate.
   *
   * @param connection Network connection between two federates.
   * @param coordination One of CoordinationType.DECENTRALIZED or CoordinationType.CENTRALIZED.
   * @param resource The resource from which the ECore model was derived.
   */
  private static void addNetworkSenderReactor(
      FedConnectionInstance connection,
      CoordinationType coordination,
      Resource resource,
      MessageReporter messageReporter) {
    LfFactory factory = LfFactory.eINSTANCE;
    var extension =
        FedTargetExtensionFactory.getExtension(connection.srcFederate.targetConfig.target);
    // Assume all the types are the same, so just use the first on the right.

    Reactor sender = getNetworkSenderReactor(connection, coordination, resource, messageReporter);

    Instantiation networkInstance = factory.createInstantiation();

    VarRef sourceRef = factory.createVarRef(); // out port from federate
    VarRef instRef = factory.createVarRef(); // out port from federate

    Reactor top =
        connection
            .getSourcePortInstance()
            .getParent()
            .getParent()
            .reactorDefinition; // Top-level reactor.

    networkInstance.setReactorClass(sender);
    networkInstance.setName(
        ASTUtils.getUniqueIdentifier(top, "ns_" + connection.getDstFederate().name));
    top.getInstantiations().add(networkInstance);

    extension.supplySenderIndexParameter(
        networkInstance, connection.getSrcFederate().networkIdSender++);
    addLevelAttribute(
        networkInstance, connection.getSourcePortInstance(), getSrcIndex(connection), connection);

    Connection senderToReaction = factory.createConnection();

    // Establish references to the involved ports.
    sourceRef.setContainer(connection.getSourcePortInstance().getParent().getDefinition());
    sourceRef.setVariable(connection.getSourcePortInstance().getDefinition());
    instRef.setContainer(networkInstance);
    instRef.setVariable(sender.getInputs().get(0));

    senderToReaction.getLeftPorts().add(sourceRef);
    senderToReaction.getRightPorts().add(instRef);

    // The connection is 'physical' if it uses the ~> notation.
    if (connection.getDefinition().isPhysical()) {
      connection.srcFederate.outboundP2PConnections.add(connection.dstFederate);
    } else {
      // If the connection is logical but coordination
      // is decentralized, we would need
      // to make P2P connections
      if (coordination == CoordinationType.DECENTRALIZED) {
        connection.srcFederate.outboundP2PConnections.add(connection.dstFederate);
      }
    }

    connection.srcFederate.networkConnections.add(senderToReaction);
    connection.srcFederate.networkSenderInstantiations.add(networkInstance);
    connection.srcFederate.networkPortToInstantiation.put(
        connection.getSourcePortInstance(), networkInstance);
  }

  /**
   * Add a network control reaction for a given output port 'source' to source's parent reactor.
   * This reaction will send a port absent message if the status of the output port is absent.
   *
   * @param connection The federated connection being replaced.
   */
  private static void addPortAbsentReaction(FedConnectionInstance connection) {
    LfFactory factory = LfFactory.eINSTANCE;
    Reaction reaction = factory.createReaction();
    Reactor top = networkSenderReactors.getOrDefault(connection, null);

    // Add the output from the contained reactor as a source to
    // the reaction to preserve precedence order.
    VarRef newPortRef = factory.createVarRef();
    newPortRef.setVariable(top.getInputs().get(0));
    reaction.getSources().add(newPortRef);

    // If the sender or receiver is in a bank of reactors, then we want
    // these reactions to appear only in the federate whose bank ID matches.
    setReactionBankIndex(reaction, connection.getSrcBank());

    // FIXME: do not create a new extension every time it is used
    FedTargetExtensionFactory.getExtension(connection.srcFederate.targetConfig.target)
        .annotateReaction(reaction);

    // Generate the code
    reaction.setCode(factory.createCode());

    reaction
        .getCode()
        .setBody(
            FedTargetExtensionFactory.getExtension(connection.srcFederate.targetConfig.target)
                .generatePortAbsentReactionBody(newPortRef, connection));

    // Insert the newly generated reaction after the generated sender and
    // receiver top-level reactions.
    top.getReactions().add(reaction);

    // Add the network output control reaction to the federate instance's list
    // of network reactions
    connection.srcFederate.networkSenderReactions.add(reaction);
    connection.srcFederate.portAbsentReactions.add(reaction);
  }
}
