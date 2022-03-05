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

package org.lflang.federated;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.stream.Collectors;

import org.eclipse.emf.ecore.util.EcoreUtil;
import org.lflang.ASTUtils;
import org.lflang.InferredType;
import org.lflang.ASTUtils;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.TimeValue;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.PortInstance;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.Connection;
import org.lflang.lf.Delay;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Parameter;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.Type;
import org.lflang.lf.Value;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;

/**
 * A helper class for AST transformations needed for federated
 * execution.
 * 
 * @author Soroush Bateni {soroush@utdallas.edu}
 * @author Edward A. Lee {eal@berkeley.edu}
 *
 */
public class FedASTUtils {

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
     * @param connection A connection between to federates.
     * @param serializer The serializer used on the connection.
     * @param type The type of the source port (indicating the type of
     *  data to be received).
     * @param networkBufferType The type of the buffer used for network
     *  communication in the target (e.g., uint8_t* in C).
     * @return The newly created action.
     */
    private static Action createNetworkAction(
        Connection connection,
        SupportedSerializers serializer,
        Type type,
        String networkBufferType
    ) {
        Reactor top = (Reactor) connection.eContainer();
        LfFactory factory = LfFactory.eINSTANCE;

        Action action = factory.createAction();
        // Name the newly created action; set its delay and type.
        action.setName(ASTUtils.getUniqueIdentifier(top, "networkMessage"));
        if (serializer == SupportedSerializers.NATIVE) {
            action.setType(type);
        } else {
            Type action_type = factory.createType();
            action_type.setId(networkBufferType);
            action.setType(action_type);
        }
        
        // The connection is 'physical' if it uses the ~> notation.
        if (connection.isPhysical()) {
            action.setOrigin(ActionOrigin.PHYSICAL);
            // Messages sent on physical connections do not
            // carry a timestamp, or a delay. The delay
            // provided using after is enforced by setting
            // the minDelay.
            if (connection.getDelay() != null) {
                action.setMinDelay(factory.createValue());
                action.getMinDelay().setTime(connection.getDelay().getTime());
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
     * 'networkAction' will contain the actual message that is sent by the sender
     * in 'action->value'. This value is forwarded to 'destination' in the network
     * receiver reaction.
     * 
     * @note: Used in federated execution
     * 
     * @param networkAction The network action (also, @see createNetworkAction)
     * @param source The source port instance.
     * @param destination The destination port instance.
     * @param connection The network connection.
     * @param sourceFederate The source federate.
     * @param destinationFederate The destination federate.
     * @param rightBankIndex The right bank index or -1 if the right reactor is not in a bank.
     * @param rightChannelIndex The right channel index or -1 if the right port is not a multiport.
     * @param generator The GeneratorBase instance used to perform some target-specific actions
     * @param coordination One of CoordinationType.DECENTRALIZED or CoordinationType.CENTRALIZED.
     * @param serializer The serializer used on the connection
     */
    private static void addNetworkReceiverReaction(
            Action networkAction,
            PortInstance source,
            PortInstance destination,
            Connection connection, 
            FederateInstance sourceFederate,
            FederateInstance destinationFederate,
            int rightBankIndex,
            int rightChannelIndex,
            GeneratorBase generator,
            CoordinationType coordination,
            SupportedSerializers serializer
    ) {
        LfFactory factory = LfFactory.eINSTANCE;
        VarRef sourceRef = factory.createVarRef();
        VarRef destRef = factory.createVarRef();
        Reactor parent = (Reactor)connection.eContainer();
        Reaction networkReceiverReaction = factory.createReaction();

        // These reactions do not require any dependency relationship
        // to other reactions in the container.
        generator.makeUnordered(networkReceiverReaction);
        
        // If the sender or receiver is in a bank of reactors, then we want
        // these reactions to appear only in the federate whose bank ID matches.
        generator.setReactionBankIndex(networkReceiverReaction, rightBankIndex);
        
        // The connection is 'physical' if it uses the ~> notation.
        if (connection.isPhysical()) {
            destinationFederate.inboundP2PConnections.add(sourceFederate);
        } else {
            // If the connection is logical but coordination
            // is decentralized, we would need
            // to make P2P connections
            if (coordination == CoordinationType.DECENTRALIZED) {
                destinationFederate.inboundP2PConnections.add(sourceFederate);
            }
        }
        
        // Record this action in the right federate.
        // The ID of the receiving port (rightPort) is the position
        // of the action in this list.
        int receivingPortID = destinationFederate.networkMessageActions.size();
        
        // Establish references to the involved ports.
        sourceRef.setContainer(source.getParent().getDefinition());
        sourceRef.setVariable(source.getDefinition());
        destRef.setContainer(destination.getParent().getDefinition());
        destRef.setVariable(destination.getDefinition());
        
        if (!connection.isPhysical()) {            
            // If the connection is not physical,
            // add the original output port of the source federate
            // as a trigger to keep the overall dependency structure. 
            // This is useful when assigning levels.
            VarRef senderOutputPort = factory.createVarRef();
            senderOutputPort.setContainer(source.getParent().getDefinition());
            senderOutputPort.setVariable(source.getDefinition());
            networkReceiverReaction.getTriggers().add(senderOutputPort);
            // Add this trigger to the list of disconnected network reaction triggers
            destinationFederate.remoteNetworkReactionTriggers.add(senderOutputPort);
        }
        
        // Add the input port at the receiver federate reactor as an effect
        networkReceiverReaction.getEffects().add(destRef);
        
        VarRef triggerRef = factory.createVarRef();
        // Establish references to the action.
        triggerRef.setVariable(networkAction);
        // Add the action as a trigger to the receiver reaction
        networkReceiverReaction.getTriggers().add(triggerRef);
        
        // Generate code for the network receiver reaction
        networkReceiverReaction.setCode(factory.createCode());
        networkReceiverReaction.getCode().setBody(generator.generateNetworkReceiverBody(
            networkAction,
            sourceRef,
            destRef,
            receivingPortID,
            sourceFederate,
            destinationFederate,
            rightBankIndex,
            rightChannelIndex,
            ASTUtils.getInferredType(networkAction),
            connection.isPhysical(),
            serializer
        ));
        
        // Add the receiver reaction to the parent
        parent.getReactions().add(networkReceiverReaction);
        
        // Add the network receiver reaction to the federate instance's list
        // of network reactions
        destinationFederate.networkReactions.add(networkReceiverReaction);
    }
    
    /**
     * Add a network control reaction for a given input port 'destination' to
     * destination's parent reactor. This reaction will block for
     * any valid logical time until it is known whether the trigger for the
     * action corresponding to the given port is present or absent.
     * 
     * @note Used in federated execution
     *
     * @param source The output port of the source federate reactor.
     *  Added as a trigger to the network control reaction to preserve the 
     *  overall dependency structure of the program across federates.
     * @param destination The input port of the destination federate reactor.
     * @param recevingPortID The ID of the receiving port
     * @param bankIndex The bank index of the receiving federate, or -1 if not in a bank.
     * @param instance The federate instance is used to keep track of all
     *  network input ports globally
     * @param generator The GeneratorBase instance used to perform some target-specific actions
     */
    private static void addNetworkInputControlReaction(
            PortInstance source,
            PortInstance destination,
            int recevingPortID,
            int bankIndex,
            FederateInstance instance,
            GeneratorBase generator
    ) {
        LfFactory factory = LfFactory.eINSTANCE;
        Reaction reaction = factory.createReaction();
        VarRef sourceRef = factory.createVarRef();
        VarRef destRef = factory.createVarRef();
        
        // If the sender or receiver is in a bank of reactors, then we want
        // these reactions to appear only in the federate whose bank ID matches.
        generator.setReactionBankIndex(reaction, bankIndex);

        // Create a new action that will be used to trigger the
        // input control reactions.
        Action newTriggerForControlReactionInput = factory.createAction();
        newTriggerForControlReactionInput.setOrigin(ActionOrigin.LOGICAL);

        // Set the container and variable according to the network port
        destRef.setContainer(destination.getParent().getDefinition());
        destRef.setVariable(destination.getDefinition());
        sourceRef.setContainer(source.getParent().getDefinition());
        sourceRef.setVariable(source.getDefinition());
        
        Reactor top = destination.getParent().getParent().reactorDefinition;
        
        newTriggerForControlReactionInput.setName(
                ASTUtils.getUniqueIdentifier(top, "inputControlReactionTrigger"));

        // Add the newly created Action to the action list of the federated reactor.
        top.getActions().add(newTriggerForControlReactionInput);

        // Create the trigger for the reaction
        VarRef newTriggerForControlReaction = factory.createVarRef();
        newTriggerForControlReaction.setVariable(newTriggerForControlReactionInput);
        
        // Add the appropriate triggers to the list of triggers of the reaction
        reaction.getTriggers().add(newTriggerForControlReaction);
        
        // Add the original output port of the source federate
        // as a trigger to keep the overall dependency structure. 
        // This is useful when assigning levels.
        reaction.getTriggers().add(sourceRef);
        // Add this trigger to the list of disconnected network reaction triggers
        instance.remoteNetworkReactionTriggers.add(sourceRef);
        
        // Add the destination port as an effect of the reaction
        reaction.getEffects().add(destRef);
        
        // Generate code for the network input control reaction
        reaction.setCode(factory.createCode());

        TimeValue maxSTP = findMaxSTP(
                destination.getDefinition(),
                instance,
                generator, 
                destination.getParent().reactorDefinition
        );

        reaction.getCode()
                .setBody(generator.generateNetworkInputControlReactionBody(
                        recevingPortID, maxSTP));
        
        generator.makeUnordered(reaction);

        // Insert the reaction
        top.getReactions().add(reaction);
        
        // Add the trigger for this reaction to the list of triggers, used to actually
        // trigger the reaction at the beginning of each logical time.
        instance.networkInputControlReactionsTriggers.add(newTriggerForControlReactionInput);
        
        // Add the network input control reaction to the federate instance's list
        // of network reactions
        instance.networkReactions.add(reaction);
    }

    /**
     * Find the maximum STP offset for the given 'port'.
     * 
     * An STP offset predicate can be nested in contained reactors in
     * the federate.
     * @param port The port to generate the STP list for.
     * @param generator The GeneratorBase instance used to perform some target-specific actions
     * @param reactor The top-level reactor (not the federate reactor)
     * @return The maximum STP as a TimeValue
     */
    private static TimeValue findMaxSTP(Variable port,
            FederateInstance instance,
            GeneratorBase generator, Reactor reactor) {
        // Find a list of STP offsets (if any exists)
        List<Value> STPList = new LinkedList<>();
        
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
                r.getSources().stream().anyMatch(s -> s.getVariable() == port);
                }).collect(Collectors.toList());
        
        // Find a list of STP offsets (if any exists)
        if (generator.isFederatedAndDecentralized()) {
            for (Reaction r : safe(reactionsWithPort)) {
                if (!instance.contains(r)) {
                    continue;
                }
                // If STP offset is determined, add it
                // If not, assume it is zero
                if (r.getStp() != null) {
                    if (r.getStp().getValue().getParameter() != null) {
                        List<Instantiation> instantList = new ArrayList<>();
                        instantList.add(instance.instantiation);
                        STPList.addAll(ASTUtils.initialValue(r.getStp().getValue().getParameter(), instantList));
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
                List<Reaction> childReactionsWithPort = ASTUtils
                    .allReactions(childReactor).stream().filter(r -> r.getTriggers().stream().anyMatch(t -> {
                        if (t instanceof VarRef) {
                            // Check if the variables match
                            return ((VarRef) t)
                                    .getVariable() == childPort
                                            .getVariable();
                        } else {
                            // Not a network port (startup or shutdown)
                            return false;
                        }
                    }) || r.getSources().stream().anyMatch(s -> s.getVariable() == childPort
                            .getVariable())).collect(Collectors.toList());

                for (Reaction r : safe(childReactionsWithPort)) {
                    if (!instance.contains(r)) {
                        continue;
                    }
                    // If STP offset is determined, add it
                    // If not, assume it is zero
                    if (r.getStp() != null) {
                        if (r.getStp().getValue() instanceof Parameter) {
                            List<Instantiation> instantList = new ArrayList<>();
                            instantList.add(childPort.getContainer());
                            STPList.addAll(ASTUtils.initialValue(r.getStp().getValue().getParameter(), instantList));
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
     * Add a network sender reaction for a given input port 'source' to
     * source's parent reactor. This reaction will react to the 'source'
     * and then send a message on the network destined for the destinationFederate.
     * 
     * @note Used in federated execution
     * 
     * @param source The source port instance.
     * @param destination The destination port instance.
     * @param connection The network connection.
     * @param sourceFederate The source federate.
     * @param leftBankIndex The left bank index or -1 if the left reactor is not in a bank.
     * @param leftChannelIndex The left channel index or -1 if the left port is not a multiport.
     * @param destinationFederate The destination federate.
     * @param generator The GeneratorBase instance used to perform some target-specific actions
     * @param coordination One of CoordinationType.DECENTRALIZED or CoordinationType.CENTRALIZED.
     * @param serializer The serializer used on the connection
     */
    private static void addNetworkSenderReaction(
            PortInstance source,
            PortInstance destination,
            Connection connection, 
            FederateInstance sourceFederate,
            int leftBankIndex,
            int leftChannelIndex,
            FederateInstance destinationFederate,
            GeneratorBase generator,
            CoordinationType coordination,
            SupportedSerializers serializer
    ) {
        LfFactory factory = LfFactory.eINSTANCE;
        // Assume all the types are the same, so just use the first on the right.
        Type type = EcoreUtil.copy(source.getDefinition().getType());
        VarRef sourceRef = factory.createVarRef();
        VarRef destRef = factory.createVarRef();
        Reactor parent = (Reactor)connection.eContainer();
        Reaction networkSenderReaction = factory.createReaction();

        // These reactions do not require any dependency relationship
        // to other reactions in the container.
        generator.makeUnordered(networkSenderReaction);
        
        // If the sender or receiver is in a bank of reactors, then we want
        // these reactions to appear only in the federate whose bank ID matches.
        generator.setReactionBankIndex(networkSenderReaction, leftBankIndex);
        
        // The connection is 'physical' if it uses the ~> notation.
        if (connection.isPhysical()) {
            sourceFederate.outboundP2PConnections.add(destinationFederate);
        } else {
            // If the connection is logical but coordination
            // is decentralized, we would need
            // to make P2P connections
            if (coordination == CoordinationType.DECENTRALIZED) {
                sourceFederate.outboundP2PConnections.add(destinationFederate);
            }
        }
        
        // Record this action in the right federate.
        // The ID of the receiving port (rightPort) is the position
        // of the action in this list.
        int receivingPortID = destinationFederate.networkMessageActions.size();


        // Establish references to the involved ports.
        sourceRef.setContainer(source.getParent().getDefinition());
        sourceRef.setVariable(source.getDefinition());
        destRef.setContainer(destination.getParent().getDefinition());
        destRef.setVariable(destination.getDefinition());
        
        // Configure the sending reaction.
        networkSenderReaction.getTriggers().add(sourceRef);
        networkSenderReaction.setCode(factory.createCode());
        networkSenderReaction.getCode().setBody(generator.generateNetworkSenderBody(
            sourceRef,
            destRef,
            receivingPortID,
            sourceFederate,
            leftBankIndex,
            leftChannelIndex,
            destinationFederate,
            InferredType.fromAST(type),
            connection.isPhysical(),
            connection.getDelay(),
            serializer
        ));
              
        // Add the sending reaction to the parent.
        parent.getReactions().add(networkSenderReaction);
        
        // Add the network sender reaction to the federate instance's list
        // of network reactions
        sourceFederate.networkReactions.add(networkSenderReaction);
    }

    /**
     * Add a network control reaction for a given output port 'source' to 
     * source's parent reactor. This reaction will send a port absent
     * message if the status of the output port is absent.
     * 
     * @note Used in federated execution
     * 
     * @param source The output port of the source federate
     * @param instance The federate instance is used to keep track of all
     *  network reactions and some relevant triggers
     * @param receivingPortID The ID of the receiving port
     * @param channelIndex The channel index of the sending port, if it is a multiport.
     * @param bankIndex The bank index of the sending federate, if it is a bank.
     * @param receivingFedID The ID of destination federate.
     * @param generator The GeneratorBase instance used to perform some target-specific actions
     * @param delay The delay value imposed on the connection using after
     */
    private static void addNetworkOutputControlReaction(
            PortInstance source,
            FederateInstance instance,
            int receivingPortID, 
            int bankIndex, 
            int channelIndex, 
            int receivingFedID,
            GeneratorBase generator,
            Delay delay
    ) {
        LfFactory factory = LfFactory.eINSTANCE;
        Reaction reaction = factory.createReaction();
        Reactor top = source.getParent().getParent().reactorDefinition; // Top-level reactor.
        
        // If the sender or receiver is in a bank of reactors, then we want
        // these reactions to appear only in the federate whose bank ID matches.
        generator.setReactionBankIndex(reaction, bankIndex);

        // Add the output from the contained reactor as a source to
        // the reaction to preserve precedence order.
        VarRef newPortRef = factory.createVarRef();
        newPortRef.setContainer(source.getParent().getDefinition());
        newPortRef.setVariable(source.getDefinition());
        reaction.getSources().add(newPortRef);

        // We use an action at the top-level to manually
        // trigger output control reactions. That action is created once
        // and recorded in the federate instance.
        // Check whether the action already has been created.
        if (instance.networkOutputControlReactionsTrigger == null) {
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
                instance.networkOutputControlReactionsTrigger 
                        = newTriggerForControlReactionVariable;
            } else {
                // If the "outputControlReactionTrigger" trigger is already
                // there, we can re-use it for this new reaction since a single trigger
                // will trigger
                // all network output control reactions.
                instance.networkOutputControlReactionsTrigger = optTriggerInput.get();
            }
        }

        // Add the trigger for all output control reactions to the list of triggers
        VarRef triggerRef = factory.createVarRef();
        triggerRef.setVariable(instance.networkOutputControlReactionsTrigger);
        reaction.getTriggers().add(triggerRef);
        
        // Generate the code
        reaction.setCode(factory.createCode());

        reaction.getCode().setBody(
                generator.generateNetworkOutputControlReactionBody(newPortRef,
                        receivingPortID, receivingFedID, bankIndex, channelIndex, delay));
        
        // Make the reaction unordered w.r.t. other reactions in the top level.
        generator.makeUnordered(reaction);

        // Insert the newly generated reaction after the generated sender and
        // receiver top-level reactions.
        top.getReactions().add(reaction);
        
        // Add the network output control reaction to the federate instance's list
        // of network reactions
        instance.networkReactions.add(reaction);
    }
    
    /** 
     * Replace the specified connection with communication between federates.
     * @param source The source port instance.
     * @param destination The destination port instance.
     * @param connection The connection.
     * @param sourceFederate The source federate.
     * @param leftBankIndex The left bank index or -1 if the left reactor is not in a bank.
     * @param leftChannelIndex The left channel index or -1 if the left port is not a multiport.
     * @param destinationFederate The destination federate.
     * @param rightBankIndex The right bank index or -1 if the right reactor is not in a bank.
     * @param rightChannelIndex The right channel index or -1 if the right port is not a multiport.
     * @param generator The GeneratorBase instance used to perform some target-specific actions
     * @param coordination One of CoordinationType.DECENTRALIZED or CoordinationType.CENTRALIZED.
     */
    public static void makeCommunication(
            PortInstance source,
            PortInstance destination,
            Connection connection, 
            FederateInstance sourceFederate,
            int leftBankIndex,
            int leftChannelIndex,
            FederateInstance destinationFederate,
            int rightBankIndex,
            int rightChannelIndex,
            GeneratorBase generator,
            CoordinationType coordination
    ) {        
        // Get the serializer
        var serializer = SupportedSerializers.NATIVE;
        if (connection.getSerializer() != null) {
            serializer = SupportedSerializers.valueOf(
                    connection.getSerializer().getType().toUpperCase()
            );
        }
        // Add it to the list of enabled serializers
        generator.enabledSerializers.add(serializer);
        
        // Add the sender reaction.
        addNetworkSenderReaction(
                source,
                destination,
                connection, 
                sourceFederate,
                leftBankIndex, 
                leftChannelIndex, 
                destinationFederate,
                generator, 
                coordination, 
                serializer
        );
        
        if (!connection.isPhysical()) {
            
            // The ID of the receiving port (rightPort) is the position
            // of the networkAction (see below) in this list.
            int receivingPortID = destinationFederate.networkMessageActions.size();
            
            // Add the network output control reaction to the parent
            FedASTUtils.addNetworkOutputControlReaction(
                source,
                sourceFederate,
                receivingPortID,
                leftBankIndex,
                leftChannelIndex,
                destinationFederate.id,
                generator,
                connection.getDelay()
            );
            
            // Add the network input control reaction to the parent
            FedASTUtils.addNetworkInputControlReaction(
                source,
                destination,
                receivingPortID,
                rightBankIndex,
                destinationFederate,
                generator
            );
        }

        // Create the network action (@see createNetworkAction)
        Action networkAction = createNetworkAction(
            connection,
            serializer,
            EcoreUtil.copy(source.getDefinition().getType()),
            generator.getNetworkBufferType());

        // Keep track of this action in the destination federate.
        destinationFederate.networkMessageActions.add(networkAction);

        // Add the action definition to the parent reactor.
        ((Reactor)connection.eContainer()).getActions().add(networkAction);

        // Add the network receiver reaction in the destinationFederate
        addNetworkReceiverReaction(
                networkAction,
                source, 
                destination, 
                connection, 
                sourceFederate,
                destinationFederate,
                rightBankIndex, 
                rightChannelIndex, 
                generator, 
                coordination,
                serializer
         );
    }
}
