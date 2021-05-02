/*************
 * Copyright (c) 2021, The University of California at Berkeley.
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
import java.util.Optional;
import java.util.stream.Collectors;

import org.lflang.ASTUtils;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.TimeValue;
import org.lflang.generator.FederateInstance;
import org.lflang.generator.GeneratorBase;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.Connection;
import org.lflang.lf.Delay;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Parameter;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.TimeUnit;
import org.lflang.lf.Type;
import org.lflang.lf.Value;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;

/**
 * A helper class for AST transformations needed for federated
 * execution.
 * 
 * @author Soroush Bateni {soroush@utdallas.edu}
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
        return list == null ? Collections.<E>emptyList() : list;
    }

    /**
     * Add a network control reaction for a given input port "portRef" to the
     * reaction queue of the federated reactor. This reaction will block for
     * any valid logical time until it is known whether the trigger for the
     * action corresponding to the given port is present or absent.
     * 
     * @note Used in federated execution
     * 
     * @param portRef The network input port
     * @param receivingPortID The ID of the receiving port
     * @param instance The federate instance is used to keep track of all
     *  network input ports globally
     * @param parent The federated reactor
     * @param generator The GeneratorBase instance used to identify certain
     *  target properties
     */
    public static void addNetworkInputControlReaction(
            VarRef port,
            int recevingPortID,
            FederateInstance instance,
            Reactor parent,
            GeneratorBase generator
        ) {
        LfFactory factory = LfFactory.eINSTANCE;
        Reaction reaction = factory.createReaction();
        VarRef newPortRef = factory.createVarRef();        
        Type portType = ASTUtils.getCopy(((Port)port.getVariable()).getType());
        
        // Create a new Input port for the reaction trigger.
        Input newTriggerForControlReactionInput = factory.createInput();       

        // Set the container and variable according to the network port
        newPortRef.setContainer(port.getContainer());
        newPortRef.setVariable(port.getVariable());
        
        newTriggerForControlReactionInput.setName(ASTUtils.getUniqueIdentifier(parent, "inputControlReactionTrigger"));
        newTriggerForControlReactionInput.setType(portType);         

        // Add the newly created Input to the input list of inputs of the federated reactor
        parent.getInputs().add(newTriggerForControlReactionInput);

        // Create the trigger for the reaction
        VarRef newTriggerForControlReaction = (VarRef) factory
                .createVarRef();
        newTriggerForControlReaction
                .setVariable(newTriggerForControlReactionInput);
        
        // Add the trigger to the list of triggers of the reaction
        reaction.getTriggers().add(newTriggerForControlReaction);
        // Add the network port as an effect of the reaction
        reaction.getEffects().add(newPortRef);
        reaction.setCode(factory.createCode());

        TimeValue maxSTP = findMaxSTP(
                newPortRef.getVariable(),
                instance,
                generator, 
                (Reactor)newPortRef.getVariable().eContainer()
                );

        reaction.getCode()
                .setBody(generator.generateNetworkInputControlReactionBody(
                        recevingPortID, maxSTP));
        
        generator.makeUnordered(reaction);

        // Insert the reaction
        parent.getReactions().add(reaction);
        
        // Add the trigger for this reaction to the list of triggers, used to actually
        // trigger the reaction at the beginning of each logical time.
        instance.networkInputControlReactionsTriggers.add(newTriggerForControlReactionInput);
    }

    /**
     * Find the maximum STP offset for the given port.
     * 
     * An STP offset predicate can be nested in contained reactors in
     * the federate.
     * @param port The port to generate the STP list for.
     * @param generator The instance of GeneratorBase
     * @param reactor The top-level reactor (not the federate reactor)
     * @return
     */
    private static TimeValue findMaxSTP(Variable port,
            FederateInstance instance,
            GeneratorBase generator, Reactor reactor) {
        // Find a list of STP offsets (if any exists)
        List<Value> STPList = new LinkedList<Value>();
        
        // First, check if there are any connections to contained reactors that
        // need to be handled
        List<Connection> connectionsWithPort = ASTUtils
                .allConnections(reactor).stream().filter(c -> {
                    return c.getLeftPorts().stream().anyMatch((VarRef v) -> {
                        return v.getVariable().equals(port);
                    });
                }).collect(Collectors.toList());


        // Find the list of reactions that have the port as trigger or source
        // (could be a variable name)
        List<Reaction> reactionsWithPort = ASTUtils
                .allReactions(reactor).stream().filter(r -> {
                    return (// Check the triggers of reaction r first
                    r.getTriggers().stream().anyMatch(t -> {
                        if (t instanceof VarRef) {
                            // Check if the variables match
                            return ((VarRef) t).getVariable() == port;
                        } else {
                            // Not a network port (startup or shutdown)
                            return false;
                        }
                    }) || // Then check the sources of reaction r
                    r.getSources().stream().anyMatch(s -> {
                        return s.getVariable() == port;
                    }));
                }).collect(Collectors.toList());
        
        // Find a list of STP offsets (if any exists)
        if (generator.isFederatedAndDecentralized()) {
            for (Reaction r : safe(reactionsWithPort)) {
                if (!instance.containsReaction(reactor, r)) {
                    continue;
                }
                // If STP offset is determined, add it
                // If not, assume it is zero
                if (r.getStp() != null) {
                    if (r.getStp().getValue().getParameter() != null) {
                        List<Instantiation> instantList = new ArrayList<Instantiation>();
                        instantList.add(instance.instantiation);
                        STPList.addAll(ASTUtils.initialValue((Parameter)r.getStp().getValue().getParameter(), instantList));
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
                        .allReactions(childReactor).stream().filter(r -> {
                            return (r.getTriggers().stream().anyMatch(t -> {
                                if (t instanceof VarRef) {
                                    // Check if the variables match
                                    return ((VarRef) t)
                                            .getVariable() == childPort
                                                    .getVariable();
                                } else {
                                    // Not a network port (startup or shutdown)
                                    return false;
                                }
                            }) || r.getSources().stream().anyMatch(s -> {
                                return s.getVariable() == childPort
                                        .getVariable();
                            }));
                        }).collect(Collectors.toList());

                for (Reaction r : safe(childReactionsWithPort)) {
                    if (!instance.containsReaction(childReactor, r)) {
                        continue;
                    }
                    // If STP offset is determined, add it
                    // If not, assume it is zero
                    if (r.getStp() != null) {
                        if (r.getStp().getValue() instanceof Parameter) {
                            List<Instantiation> instantList = new ArrayList<Instantiation>();
                            instantList.add(childPort.getContainer());
                            STPList.addAll(ASTUtils.initialValue((Parameter)r.getStp().getValue().getParameter(), instantList));
                        } else {
                            STPList.add(r.getStp().getValue());
                        }
                    }
                }
            }
        }
        
        TimeValue maxSTP = new TimeValue(0, TimeUnit.NONE);
        for (Value value : safe(STPList)) {
            TimeValue tValue = ASTUtils.getTimeValue(value);
            if(maxSTP.isEarlierThan(tValue)) {
                maxSTP = tValue;
            }
        }
        
        return maxSTP;
    }

    /**
     * Add a network control reaction for a given output port "portRef" to the
     * reactions of the container reactor. This reaction will send a port absent
     * message if the status of the output port is absent.
     * 
     * @note Used in federated execution
     * 
     * @param portRef The output port
     * @param instance The federate instance is used to keep track of all
     *        network input ports globally
     * @param receivingPortID The ID of the receiving port
     * @param channelIndex The channel index of the sending port, if it is a multiport.
     * @param bankIndex The bank index of the sending federate, if it is a bank.
     * @param receivingFedID The ID of destination federate.
     * @param generator The GeneratorBase instance used to identify certain
     *        target properties
     * @param delay The delay value imposed on the connection using after
     */
    public static void addNetworkOutputControlReaction(
            VarRef portRef,
            FederateInstance instance,
            int receivingPortID, 
            int channelIndex, 
            int bankIndex, 
            int receivingFedID,
            GeneratorBase generator,
            Delay delay
    ) {
        LfFactory factory = LfFactory.eINSTANCE;
        Reaction reaction = factory.createReaction();
        Reactor reactor = (Reactor) portRef.eContainer().eContainer();
        VarRef newPortRef = factory.createVarRef();
        
        newPortRef.setContainer(portRef.getContainer());
        newPortRef.setVariable(portRef.getVariable());

        if (instance.networkOutputControlReactionsTrigger == null) {
            String triggerName = "outputControlReactionTrigger";

            // The trigger for the reaction
            VarRef newTriggerForControlReaction = (VarRef) factory
                    .createVarRef();

            // Find the trigger in the reactor definition, which could have been
            // generated for another federate instance
            Optional<Input> optTriggerInput = reactor.getInputs().stream()
                    .filter(I -> {
                        return I.getName().equals(triggerName);
                    }).findFirst();

            if (optTriggerInput.isEmpty()) {
                // If no trigger with the name "outputControlReactionTrigger" is
                // already added
                // to the reactor definition, we need to create it for the first
                // time.
                Variable newTriggerForControlReactionVariable = factory
                        .createInput();
                newTriggerForControlReactionVariable.setName(triggerName);

                Type portType = factory.createType();
                portType.setId(generator.getTargetTimeType());
                ((Input) newTriggerForControlReactionVariable)
                        .setType(portType);

                newTriggerForControlReaction
                        .setVariable(newTriggerForControlReactionVariable);

                reactor.getInputs()
                        .add((Input) newTriggerForControlReactionVariable);
            } else {
                // If the "outputControlReactionTrigger" trigger is already
                // there, we
                // can re-use it for this new reaction since a single trigger
                // will trigger
                // all network output control reactions.
                newTriggerForControlReaction.setVariable(optTriggerInput.get());
            }
            instance.networkOutputControlReactionsTrigger = newTriggerForControlReaction;
        }

        // Add the trigger for all output control reactions to the list of triggers
        reaction.getTriggers()
                .add(instance.networkOutputControlReactionsTrigger);
        // Add the output from the contained reactor as a source to preserve
        // precedence order
        reaction.getSources().add(newPortRef);
        
        // Generate the code
        reaction.setCode(factory.createCode());

        reaction.getCode().setBody(
                generator.generateNetworkOutputControlReactionBody(newPortRef,
                        receivingPortID, receivingFedID, bankIndex, channelIndex, delay));
        
        generator.makeUnordered(reaction);

        // Insert the newly generated reaction after the generated sender and
        // receiver
        // top-level reactions
        reactor.getReactions().add(reaction);
    }
    
    /** 
     * Replace the specified connection with a communication between federates.
     * @param connection The connection.
     * @param leftFederate The source federate.
     * @param rightFederate The destination federate.
     */
    public static void makeCommunication(
        Connection connection, 
        FederateInstance leftFederate,
        int leftBankIndex,
        int leftChannelIndex,
        FederateInstance rightFederate,
        int rightBankIndex,
        int rightChannelIndex,
        GeneratorBase generator,
        CoordinationType coordination
    ) {
        LfFactory factory = LfFactory.eINSTANCE;
        // Assume all the types are the same, so just use the first on the right.
        Type type = ASTUtils.getCopy(((Port)connection.getRightPorts().get(0).getVariable()).getType());
        Action action = factory.createAction();
        VarRef triggerRef = factory.createVarRef();
        VarRef inRef = factory.createVarRef();
        VarRef outRef = factory.createVarRef();
        Reactor parent = (Reactor)connection.eContainer();
        Reaction r1 = factory.createReaction();
        Reaction r2 = factory.createReaction();
        
        // These reactions do not require any dependency relationship
        // to other reactions in the container.
        generator.makeUnordered(r1);
        generator.makeUnordered(r2);

        // Name the newly created action; set its delay and type.
        action.setName(ASTUtils.getUniqueIdentifier(parent, "networkMessage"));
        action.setType(type);
        
        // The connection is 'physical' if it uses the ~> notation.
        if (connection.isPhysical()) {
            leftFederate.outboundP2PConnections.add(rightFederate);
            rightFederate.inboundP2PConnections.add(leftFederate);
            action.setOrigin(ActionOrigin.PHYSICAL);
            // Messages sent on physical connections do not
            // carry a timestamp, or a delay. The delay
            // provided using after is enforced by setting
            // the minDelay.
            if (connection.getDelay() != null) {
                action.setMinDelay(factory.createValue());
                action.getMinDelay().setTime(factory.createTime());
                action.getMinDelay().getTime().setInterval(connection.getDelay().getInterval());
                action.getMinDelay().getTime().setUnit(connection.getDelay().getUnit());
            }
        } else {
            // If the connection is logical but coordination
            // is decentralized, we would need
            // to make P2P connections
            if (coordination == CoordinationType.DECENTRALIZED) {
                leftFederate.outboundP2PConnections.add(rightFederate);
                rightFederate.inboundP2PConnections.add(leftFederate);               
            }            
            action.setOrigin(ActionOrigin.LOGICAL);
        }
        
        // Record this action in the right federate.
        // The ID of the receiving port (rightPort) is the position
        // of the action in this list.
        int receivingPortID = rightFederate.networkMessageActions.size();
        rightFederate.networkMessageActions.add(action);

        // Establish references to the action.
        triggerRef.setVariable(action);

        // Establish references to the involved ports.
        // FIXME: This does not support parallel connections yet!!
        if (connection.getLeftPorts().size() > 1 || connection.getRightPorts().size() > 1) {
            throw new UnsupportedOperationException("FIXME: " +
                                                    "Parallel connections are not yet supported between federates.");
        }
        inRef.setContainer(connection.getLeftPorts().get(0).getContainer());
        inRef.setVariable(connection.getLeftPorts().get(0).getVariable());
        outRef.setContainer(connection.getRightPorts().get(0).getContainer());
        outRef.setVariable(connection.getRightPorts().get(0).getVariable());

        // Add the action to the reactor.
        parent.getActions().add(action);
        
        // Configure the sending reaction.
        r1.getTriggers().add(inRef);
        r1.setCode(factory.createCode());
        r1.getCode().setBody(generator.generateNetworkSenderBody(
            inRef,
            outRef,
            receivingPortID,
            leftFederate,
            leftBankIndex,
            leftChannelIndex,
            rightFederate,
            ASTUtils.getInferredType(action),
            connection.isPhysical(),
            connection.getDelay()
        ));
              
        // Add the sending reaction to the parent.
        parent.getReactions().add(r1);
        
        if (!connection.isPhysical()) {           
            // Add the network output control reaction to the parent
            FedASTUtils.addNetworkOutputControlReaction(
                connection.getLeftPorts().get(0), 
                leftFederate,
                receivingPortID,
                leftBankIndex,
                leftChannelIndex,
                rightFederate.id,
                generator,
                connection.getDelay()
            );
            
            // Add the network input control reaction to the parent
            FedASTUtils.addNetworkInputControlReaction(
                connection.getRightPorts().get(0),
                receivingPortID,
                rightFederate,
                parent,
                generator
            );
        }        


        // Configure the receiving reaction.
        r2.getTriggers().add(triggerRef);
        r2.getEffects().add(outRef);
        r2.setCode(factory.createCode());
        r2.getCode().setBody(generator.generateNetworkReceiverBody(
            action,
            inRef,
            outRef,
            receivingPortID,
            leftFederate,
            rightFederate,
            rightBankIndex,
            rightChannelIndex,
            ASTUtils.getInferredType(action),
            connection.isPhysical()
        ));
        
        // Add the receiver reaction to the parent
        parent.getReactions().add(r2);
    }

}
