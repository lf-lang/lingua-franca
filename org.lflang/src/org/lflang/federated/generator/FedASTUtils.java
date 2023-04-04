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

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import org.eclipse.emf.ecore.resource.Resource;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.InferredType;
import org.lflang.ModelInfo;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.TimeValue;
import org.lflang.federated.extensions.FedTargetExtensionFactory;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.BuiltinTriggerRef;
import org.lflang.lf.Connection;
import org.lflang.lf.Expression;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Model;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.Type;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;

import com.google.common.collect.Iterators;

/**
 * A helper class for AST transformations needed for federated
 * execution.
 *
 * @author Soroush Bateni
 * @author Edward A. Lee
 */
public class FedASTUtils {

    /**
     * Map from reactions to bank indices
     */
    private static Map<Reaction,Integer> reactionBankIndices = null;

    /**
     * Mark the specified reaction to belong to only the specified
     * bank index. This is needed because reactions cannot declare
     * a specific bank index as an effect or trigger. Reactions that
     * send messages between federates, including absent messages,
     * need to be specific to a bank member.
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
            Iterators.filter(resource.getAllContents(), Reactor.class),
            Reactor::isFederated
        );
    }

    /**
     * Replace the specified connection with communication between federates.
     *
     * @param connection   Network connection between two federates.
     * @param coordination One of CoordinationType.DECENTRALIZED or
     *                     CoordinationType.CENTRALIZED.
     * @param errorReporter Used to report errors encountered.
     */
    public static void makeCommunication(
        FedConnectionInstance connection,
        CoordinationType coordination,
        ErrorReporter errorReporter
    ) {

        // Add the sender reaction.
        addNetworkSenderReaction(
            connection,
            coordination,
            errorReporter
        );

        // Next, generate control reactions
        if (
            !connection.getDefinition().isPhysical() &&
                // Connections that are physical don't need control reactions
                connection.getDefinition().getDelay()
                    == null // Connections that have delays don't need control reactions
        ) {
            // Add the network output control reaction to the parent
            FedASTUtils.addNetworkOutputControlReaction(connection);

            // Add the network input control reaction to the parent
            FedASTUtils.addNetworkInputControlReaction(connection, coordination, errorReporter);
        }

        // Create the network action (@see createNetworkAction)
        Action networkAction = createNetworkAction(connection);

        // Keep track of this action in the destination federate.
        connection.dstFederate.networkMessageActions.add(networkAction);

        // Add the action definition to the parent reactor.
        ((Reactor) connection.getDefinition().eContainer()).getActions().add(networkAction);

        // Add the network receiver reaction in the destinationFederate
        addNetworkReceiverReaction(
            networkAction,
            connection,
            coordination,
            errorReporter
        );
    }

    /**
     * Create a "network action" in the reactor that contains the given
     * connection and return it.
     *
     * The purpose of this action is to serve as a trigger for a "network
     * input reaction" that is responsible for relaying messages to the
     * port that is on the receiving side of the given connection. The
     * connection is assumed to be between two reactors that reside in
     * distinct federates. Hence, the container of the connection is
     * assumed to be top-level.
     *
     * @param connection A connection between two federates
     * @return The newly created action.
     */
    private static Action createNetworkAction(FedConnectionInstance connection) {
        Reactor top = (Reactor) connection.getDefinition().eContainer();
        LfFactory factory = LfFactory.eINSTANCE;

        Action action = factory.createAction();
        // Name the newly created action; set its delay and type.
        action.setName(ASTUtils.getUniqueIdentifier(top, "networkMessage"));
        if (connection.serializer == SupportedSerializers.NATIVE) {
            action.setType(EcoreUtil.copy(connection.getSourcePortInstance().getDefinition().getType()));
        } else {
            Type action_type = factory.createType();
            action_type.setId(
                FedTargetExtensionFactory.getExtension(
                    connection.srcFederate.target
                                   ).getNetworkBufferType()
            );
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

    /**
     * Add a network receiver reaction for a given input port 'destination' to
     * destination's parent reactor. This reaction will react to a generated
     * 'networkAction' (triggered asynchronously, e.g., by federate.c). This
     * 'networkAction' will contain the actual message that is sent by the
     * sender
     * in 'action->value'. This value is forwarded to 'destination' in the
     * network
     * receiver reaction.
     *
     * @param networkAction       The network action (also, @see createNetworkAction)
     * @param connection          FIXME
     * @param coordination        One of CoordinationType.DECENTRALIZED or
     *                            CoordinationType.CENTRALIZED.
     * @note: Used in federated execution
     */
    private static void addNetworkReceiverReaction(
        Action networkAction,
        FedConnectionInstance connection,
        CoordinationType coordination,
        ErrorReporter errorReporter
    ) {
        LfFactory factory = LfFactory.eINSTANCE;
        VarRef sourceRef = factory.createVarRef();
        VarRef destRef = factory.createVarRef();
        Reactor parent = (Reactor) connection.getDefinition().eContainer();
        Reaction networkReceiverReaction = factory.createReaction();

        // If the sender or receiver is in a bank of reactors, then we want
        // these reactions to appear only in the federate whose bank ID matches.
        setReactionBankIndex(networkReceiverReaction, connection.getDstBank());

        // FIXME: do not create a new extension every time it is used
        FedTargetExtensionFactory.getExtension(connection.srcFederate.target)
                                 .annotateReaction(networkReceiverReaction);

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
        destRef.setContainer(connection.getDestinationPortInstance().getParent().getDefinition());
        destRef.setVariable(connection.getDestinationPortInstance().getDefinition());

        // Add the input port at the receiver federate reactor as an effect
        networkReceiverReaction.getEffects().add(destRef);

        VarRef triggerRef = factory.createVarRef();
        // Establish references to the action.
        triggerRef.setVariable(networkAction);
        // Add the action as a trigger to the receiver reaction
        networkReceiverReaction.getTriggers().add(triggerRef);

        // Generate code for the network receiver reaction
        networkReceiverReaction.setCode(factory.createCode());
        networkReceiverReaction.getCode().setBody(
            FedTargetExtensionFactory.getExtension(
                connection.dstFederate.target).generateNetworkReceiverBody(
                    networkAction,
                    sourceRef,
                    destRef,
                    connection,
                    ASTUtils.getInferredType(networkAction),
                    coordination,
                    errorReporter
            ));


        ASTUtils.addReactionAttribute(networkReceiverReaction, "_unordered");

        // Add the receiver reaction to the parent
        parent.getReactions().add(networkReceiverReaction);

        // Add the network receiver reaction to the federate instance's list
        // of network reactions
        connection.dstFederate.networkReactions.add(networkReceiverReaction);


        if (
            !connection.getDefinition().isPhysical() &&
                // Connections that are physical don't need control reactions
                connection.getDefinition().getDelay()
                    == null // Connections that have delays don't need control reactions
        ) {
            // Add necessary dependencies to reaction to ensure that it executes correctly
            // relative to other network input control reactions in the federate.
            addRelativeDependency(connection, networkReceiverReaction, errorReporter);
        }
    }

    /**
     * Add a network control reaction for a given input port 'destination' to
     * destination's parent reactor. This reaction will block for
     * any valid logical time until it is known whether the trigger for the
     * action corresponding to the given port is present or absent.
     *
     * @param connection    FIXME
     * @param coordination  FIXME
     * @param errorReporter
     * @note Used in federated execution
     */
    private static void addNetworkInputControlReaction(
        FedConnectionInstance connection,
        CoordinationType coordination,
        ErrorReporter errorReporter) {

        LfFactory factory = LfFactory.eINSTANCE;
        Reaction reaction = factory.createReaction();
        VarRef destRef = factory.createVarRef();
        int receivingPortID = connection.dstFederate.networkMessageActions.size();

        // If the sender or receiver is in a bank of reactors, then we want
        // these reactions to appear only in the federate whose bank ID matches.
        setReactionBankIndex(reaction, connection.getDstBank());

        // FIXME: do not create a new extension every time it is used
        FedTargetExtensionFactory.getExtension(connection.srcFederate.target)
                                 .annotateReaction(reaction);

        // Create a new action that will be used to trigger the
        // input control reactions.
        Action newTriggerForControlReactionInput = factory.createAction();
        newTriggerForControlReactionInput.setOrigin(ActionOrigin.LOGICAL);

        // Set the container and variable according to the network port
        destRef.setContainer(connection.getDestinationPortInstance().getParent().getDefinition());
        destRef.setVariable(connection.getDestinationPortInstance().getDefinition());

        Reactor top = connection.getDestinationPortInstance()
                                .getParent()
                                .getParent().reactorDefinition;

        newTriggerForControlReactionInput.setName(
            ASTUtils.getUniqueIdentifier(top, "inputControlReactionTrigger"));

        // Add the newly created Action to the action list of the federated reactor.
        top.getActions().add(newTriggerForControlReactionInput);

        // Create the trigger for the reaction
        VarRef newTriggerForControlReaction = factory.createVarRef();
        newTriggerForControlReaction.setVariable(newTriggerForControlReactionInput);

        // Add the appropriate triggers to the list of triggers of the reaction
        reaction.getTriggers().add(newTriggerForControlReaction);

        // Add the destination port as an effect of the reaction
        reaction.getEffects().add(destRef);

        // Generate code for the network input control reaction
        reaction.setCode(factory.createCode());

        TimeValue maxSTP = findMaxSTP(connection, coordination);

        reaction.getCode()
                .setBody(
                    FedTargetExtensionFactory
                        .getExtension(connection.dstFederate.target)
                        .generateNetworkInputControlReactionBody(
                            receivingPortID,
                            maxSTP,
                            coordination
                        )
                );

        ASTUtils.addReactionAttribute(reaction, "_unordered");

        // Insert the reaction
        top.getReactions().add(reaction);

        // Add the trigger for this reaction to the list of triggers, used to actually
        // trigger the reaction at the beginning of each logical time.
        connection.dstFederate.networkInputControlReactionsTriggers.add(newTriggerForControlReactionInput);

        // Add the network input control reaction to the federate instance's list
        // of network reactions
        connection.dstFederate.networkReactions.add(reaction);

        // Add necessary dependencies to reaction to ensure that it executes correctly
        // relative to other network input control reactions in the federate.
        addRelativeDependency(connection, reaction, errorReporter);
    }

    /**
     * Add necessary dependency information to the signature of {@code networkInputReaction} so that
     * it can execute in the correct order relative to other network reactions in the federate.
     *
     * In particular, we want to avoid a deadlock if multiple network input control reactions in federate
     * are in a zero-delay cycle through other federates in the federation. To avoid the deadlock, we encode the
     * zero-delay cycle inside the federate by adding an artificial dependency from the output port of this federate
     * that is involved in the cycle to the signature of {@code networkInputReaction} as a source.
     */
    private static void addRelativeDependency(
        FedConnectionInstance connection,
        Reaction networkInputReaction,
        ErrorReporter errorReporter) {
        var upstreamOutputPortsInFederate =
            findUpstreamPortsInFederate(
                connection.dstFederate,
                connection.getSourcePortInstance(),
                new HashSet<>(),
                new HashSet<>()
            );

        ModelInfo info = new ModelInfo();
        for (var port: upstreamOutputPortsInFederate) {
            VarRef sourceRef = ASTUtils.factory.createVarRef();

            sourceRef.setContainer(port.getParent().getDefinition());
            sourceRef.setVariable(port.getDefinition());
            networkInputReaction.getSources().add(sourceRef);

            // Remove the port if it introduces cycles
            info.update(
                   (Model)networkInputReaction.eContainer().eContainer(),
                    errorReporter
                    );
            if (!info.topologyCycles().isEmpty()) {
                    networkInputReaction.getSources().remove(sourceRef);
                }
        }

    }

    /**
     * Go upstream from input port {@code port} until we reach one or more output
     * ports that belong to the same federate.
     *
     * Along the path, we follow direct connections, as well as reactions, as long
     * as there is no logical delay. When following reactions, we also follow
     * dependant reactions (because we are traversing a potential cycle backwards).
     *
     * @return A set of {@link PortInstance}. If no port exist that match the
     * criteria, return an empty set.
     */
    private static Set<PortInstance> findUpstreamPortsInFederate(
        FederateInstance federate,
        PortInstance port,
        Set<PortInstance> visitedPorts,
        Set<ReactionInstance>  reactionVisited
    ) {
        Set<PortInstance> toReturn = new HashSet<>();
        if (port == null) return toReturn;
        else if (federate.contains(port.getParent())) {
            // Reached the requested federate
            toReturn.add(port);
            visitedPorts.add(port);
        } else if (visitedPorts.contains(port)) {
            return toReturn;
        } else {
            visitedPorts.add(port);
            // Follow depends on reactions
            port.getDependsOnReactions().forEach(
                reaction -> {
                    followReactionUpstream(federate, visitedPorts, toReturn, reaction, reactionVisited);
                }
            );
            // Follow depends on ports
            port.getDependsOnPorts().forEach(
                it -> toReturn.addAll(
                    findUpstreamPortsInFederate(federate, it.instance, visitedPorts, reactionVisited)
                )
            );
        }
        return toReturn;
    }

    /**
     * Follow reactions upstream. This is part of the algorithm of
     * {@link #findUpstreamPortsInFederate(FederateInstance, PortInstance, Set)}.
     */
    private static void followReactionUpstream(
        FederateInstance federate,
        Set<PortInstance> visitedPorts,
        Set<PortInstance> toReturn,
        ReactionInstance reaction,
        Set<ReactionInstance>  reactionVisited
    ) {
        if (reactionVisited.contains(reaction)) return;
        reactionVisited.add(reaction);
        // Add triggers
        Set<VarRef> varRefsToFollow = new HashSet<>();
        varRefsToFollow.addAll(
            reaction.getDefinition()
                    .getTriggers()
                    .stream()
                    .filter(
                  trigger -> !(trigger instanceof BuiltinTriggerRef)
              )
              .map(VarRef.class::cast).toList());
        // Add sources
        varRefsToFollow.addAll(reaction.getDefinition().getSources());

        // Follow everything upstream
        varRefsToFollow.forEach(
            varRef ->
                toReturn.addAll(
                    findUpstreamPortsInFederate(
                        federate,
                        reaction.getParent()
                            .lookupPortInstance(varRef),
                        visitedPorts,
                        reactionVisited
                    )
                )
        );

        reaction.dependsOnReactions()
                .stream()
                .filter(
                    // Stay within the reactor
                    it -> it.getParent().equals(reaction.getParent())
                )
                .forEach(
                    it -> followReactionUpstream(federate, visitedPorts, toReturn, it, reactionVisited)
                );

        // FIXME: This is most certainly wrong. Please fix it.
        reaction.dependentReactions()
                .stream()
                .filter(
                    // Stay within the reactor
                    it -> it.getParent().equals(reaction.getParent())
                )
                .forEach(
                    it -> followReactionUpstream(federate, visitedPorts, toReturn, it, reactionVisited)
                );
    }

    /**
     * Find the maximum STP offset for the given 'port'.
     *
     * An STP offset predicate can be nested in contained reactors in
     * the federate.
     *
     * @param connection The connection to find the max STP offset for.
     * @param coordination The coordination scheme.
     * @return The maximum STP as a TimeValue
     */
    private static TimeValue findMaxSTP(FedConnectionInstance connection,
                                        CoordinationType coordination) {
        Variable port = connection.getDestinationPortInstance().getDefinition();
        FederateInstance instance = connection.dstFederate;
        Reactor reactor = connection.getDestinationPortInstance().getParent().reactorDefinition;

        // Find a list of STP offsets (if any exists)
        List<Expression> STPList = new LinkedList<>();

        // First, check if there are any connections to contained reactors that
        // need to be handled
        List<Connection> connectionsWithPort = ASTUtils
            .allConnections(reactor).stream().filter(c -> c.getLeftPorts()
                                                           .stream()
                                                           .anyMatch((VarRef v) -> v
                                                               .getVariable().equals(port)))
            .collect(Collectors.toList());


        // Find the list of reactions that have the port as trigger or source
        // (could be a variable name)
        List<Reaction> reactionsWithPort = ASTUtils
            .allReactions(reactor).stream().filter(r -> {
                // Check the triggers of reaction r first
                return r.getTriggers().stream().anyMatch(t -> {
                    if (t instanceof VarRef) {
                        // Check if the variables match
                        return ((VarRef) t).getVariable() == port;
                    } else {
                        // Not a network port (startup or shutdown)
                        return false;
                    }
                }) || // Then check the sources of reaction r
                    r.getSources().stream().anyMatch(s -> s.getVariable()
                        == port);
            }).collect(Collectors.toList());

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
                Reactor childReactor = (Reactor) childPort.getVariable()
                                                          .eContainer();
                // Find the list of reactions that have the port as trigger or
                // source (could be a variable name)
                List<Reaction> childReactionsWithPort =
                    ASTUtils.allReactions(childReactor)
                            .stream()
                            .filter(r ->
                                        r.getTriggers().stream().anyMatch(t -> {
                                            if (t instanceof VarRef) {
                                                // Check if the variables match
                                                return
                                                    ((VarRef) t).getVariable()
                                                        == childPort.getVariable();
                                            } else {
                                                // Not a network port (startup or shutdown)
                                                return false;
                                            }
                                        }) ||
                                            r.getSources()
                                             .stream()
                                             .anyMatch(s ->
                                                           s.getVariable()
                                                               == childPort.getVariable())
                            ).collect(Collectors.toList());

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
     * @param <E>  The type of the list
     * @param list The potentially null List
     * @return Empty list or the original list
     */
    public static <E> List<E> safe(List<E> list) {
        return list == null ? Collections.emptyList() : list;
    }

    /**
     * Add a network sender reaction for a given input port 'source' to
     * source's parent reactor. This reaction will react to the 'source'
     * and then send a message on the network destined for the
     * destinationFederate.
     *
     * @param connection   Network connection between two federates.
     * @param coordination One of CoordinationType.DECENTRALIZED or
     *                     CoordinationType.CENTRALIZED.
     * @param errorReporter FIXME
     * @note Used in federated execution
     */
    private static void addNetworkSenderReaction(
        FedConnectionInstance connection,
        CoordinationType coordination,
        ErrorReporter errorReporter
    ) {
        LfFactory factory = LfFactory.eINSTANCE;
        // Assume all the types are the same, so just use the first on the right.
        Type type = EcoreUtil.copy(connection.getSourcePortInstance().getDefinition().getType());
        VarRef sourceRef = factory.createVarRef();
        VarRef destRef = factory.createVarRef();
        Reactor parent = (Reactor) connection.getDefinition().eContainer();
        Reaction networkSenderReaction = factory.createReaction();

        // FIXME: do not create a new extension every time it is used
        FedTargetExtensionFactory.getExtension(connection.srcFederate.target)
                                 .annotateReaction(networkSenderReaction);

        // If the sender or receiver is in a bank of reactors, then we want
        // these reactions to appear only in the federate whose bank ID matches.
        setReactionBankIndex(networkSenderReaction, connection.getSrcBank());

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

        // Establish references to the involved ports.
        sourceRef.setContainer(connection.getSourcePortInstance().getParent().getDefinition());
        sourceRef.setVariable(connection.getSourcePortInstance().getDefinition());
        destRef.setContainer(connection.getDestinationPortInstance().getParent().getDefinition());
        destRef.setVariable(connection.getDestinationPortInstance().getDefinition());

        // Configure the sending reaction.
        networkSenderReaction.getTriggers().add(sourceRef);
        networkSenderReaction.setCode(factory.createCode());
        networkSenderReaction.getCode().setBody(
            FedTargetExtensionFactory.getExtension(connection.srcFederate.target)
                                     .generateNetworkSenderBody(
                                   sourceRef,
                                   destRef,
                                   connection,
                                   InferredType.fromAST(type),
                                   coordination,
                                   errorReporter
                               ));

        ASTUtils.addReactionAttribute(networkSenderReaction, "_unordered");

        // Add the sending reaction to the parent.
        parent.getReactions().add(networkSenderReaction);

        // Add the network sender reaction to the federate instance's list
        // of network reactions
        connection.srcFederate.networkReactions.add(networkSenderReaction);
    }

    /**
     * Add a network control reaction for a given output port 'source' to
     * source's parent reactor. This reaction will send a port absent
     * message if the status of the output port is absent.
     *
     * @param connection FIXME
     * @note Used in federated execution
     */
    private static void addNetworkOutputControlReaction(FedConnectionInstance connection) {
        LfFactory factory = LfFactory.eINSTANCE;
        Reaction reaction = factory.createReaction();
        Reactor top = connection.getSourcePortInstance()
                                .getParent()
                                .getParent().reactorDefinition; // Top-level reactor.

        // Add the output from the contained reactor as a source to
        // the reaction to preserve precedence order.
        VarRef newPortRef = factory.createVarRef();
        newPortRef.setContainer(connection.getSourcePortInstance().getParent().getDefinition());
        newPortRef.setVariable(connection.getSourcePortInstance().getDefinition());
        reaction.getSources().add(newPortRef);

        // If the sender or receiver is in a bank of reactors, then we want
        // these reactions to appear only in the federate whose bank ID matches.
        setReactionBankIndex(reaction, connection.getSrcBank());

        // FIXME: do not create a new extension every time it is used
        FedTargetExtensionFactory.getExtension(connection.srcFederate.target)
                                 .annotateReaction(reaction);

        // We use an action at the top-level to manually
        // trigger output control reactions. That action is created once
        // and recorded in the federate instance.
        // Check whether the action already has been created.
        if (connection.srcFederate.networkOutputControlReactionsTrigger == null) {
            // The port has not been created.
            String triggerName = "outputControlReactionTrigger";

            // Find the trigger definition in the reactor definition, which could have been
            // generated for another federate instance if there are multiple instances
            // of the same reactor that are each distinct federates.
            Optional<Action> optTriggerInput
                = top.getActions().stream().filter(
                I -> I.getName().equals(triggerName)).findFirst();

            if (optTriggerInput.isEmpty()) {
                // If no trigger with the name "outputControlReactionTrigger" is
                // already added to the reactor definition, we need to create it
                // for the first time. The trigger is a logical action.
                Action newTriggerForControlReactionVariable = factory.createAction();
                newTriggerForControlReactionVariable.setName(triggerName);
                newTriggerForControlReactionVariable.setOrigin(ActionOrigin.LOGICAL);
                top.getActions().add(newTriggerForControlReactionVariable);

                // Now that the variable is created, store it in the federate instance
                connection.srcFederate.networkOutputControlReactionsTrigger
                    = newTriggerForControlReactionVariable;
            } else {
                // If the "outputControlReactionTrigger" trigger is already
                // there, we can re-use it for this new reaction since a single trigger
                // will trigger
                // all network output control reactions.
                connection.srcFederate.networkOutputControlReactionsTrigger = optTriggerInput.get();
            }
        }

        // Add the trigger for all output control reactions to the list of triggers
        VarRef triggerRef = factory.createVarRef();
        triggerRef.setVariable(connection.srcFederate.networkOutputControlReactionsTrigger);
        reaction.getTriggers().add(triggerRef);

        // Generate the code
        reaction.setCode(factory.createCode());

        reaction.getCode().setBody(
            FedTargetExtensionFactory.getExtension(connection.srcFederate.target)
                                     .generateNetworkOutputControlReactionBody(newPortRef, connection));

        ASTUtils.addReactionAttribute(reaction, "_unordered");


        // Insert the newly generated reaction after the generated sender and
        // receiver top-level reactions.
        top.getReactions().add(reaction);

        // Add the network output control reaction to the federate instance's list
        // of network reactions
        connection.srcFederate.networkReactions.add(reaction);
    }
}
