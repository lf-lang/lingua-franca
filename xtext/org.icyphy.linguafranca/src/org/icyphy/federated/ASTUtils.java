/* A helper class for analyzing the AST. */

/*************
 * Copyright (c) 2020, The University of California at Berkeley.
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

package org.icyphy.federated;

import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

import org.eclipse.xtext.xbase.lib.CollectionLiterals;
import org.icyphy.generator.FederateInstance;
import org.icyphy.generator.GeneratorBase;
import org.icyphy.linguaFranca.Connection;
import org.icyphy.linguaFranca.Input;
import org.icyphy.linguaFranca.LinguaFrancaFactory;
import org.icyphy.linguaFranca.Port;
import org.icyphy.linguaFranca.Reaction;
import org.icyphy.linguaFranca.Reactor;
import org.icyphy.linguaFranca.Type;
import org.icyphy.linguaFranca.Value;
import org.icyphy.linguaFranca.VarRef;
import org.icyphy.linguaFranca.Variable;

/**
 * @author Soroush Bateni {soroush@utdallas.edu}
 *
 */
public class ASTUtils {

    /**
     * Return a null-safe List
     * 
     * @param <E>
     * @param list The potentially null List
     * @return Empty list or the original list
     */
    public static <E> List<E> safe(List<E> list) {
        return list == null ? Collections.<E>emptyList() : list;
    }

    /**
     * Add a network control reaction for a given input port "portRef" to the
     * reaction queue of its containing reactor. This reaction will block for
     * any valid logical time until it is known whether the trigger of that
     * given port is present or absent.
     * 
     * @note Used in federated execution
     * 
     * @input portRef The input port
     * @input instance The federate instance is used to keep track of all
     *  network input ports globally
     * @input generator The GeneratorBase instance used to identify certain
     *  target properties
     * @input isTopLevel indicates whether this reaction is being produced in the top-level
     *  federated reactor or not
     */
    public static void addNetworkInputControlReaction(VarRef portRef, int recevingPortID, Reactor reactor,
            FederateInstance instance, GeneratorBase generator, boolean isTopLevel) {
        LinguaFrancaFactory factory = LinguaFrancaFactory.eINSTANCE;
        Reaction reaction = factory.createReaction();
        VarRef newPortRef = factory.createVarRef();

        newPortRef.setContainer(null);
        newPortRef.setVariable((Input) portRef.getVariable());

        // First, check if there are any connections to contained reactors that
        // need to be handled
        List<Connection> connectionsWithPort = org.icyphy.ASTUtils
                .allConnections(reactor).stream().filter(c -> {
                    return c.getLeftPorts().stream().anyMatch((VarRef v) -> {
                        return v.getVariable().equals(portRef.getVariable());
                    });
                }).collect(Collectors.toList());

        // Find the list of reactions that have the port as trigger or source
        // (could be a variable name)
        List<Reaction> reactionsWithPort = org.icyphy.ASTUtils
                .allReactions(reactor).stream().filter(r -> {
                    return (// Check the triggers of reaction r first
                    r.getTriggers().stream().anyMatch(t -> {
                        if (t instanceof VarRef) {
                            // Check if the variables match
                            return ((VarRef) t).getVariable() == portRef
                                    .getVariable();
                        } else {
                            // Not a network port (startup or shutdown)
                            return false;
                        }
                    }) || // Then check the sources of reaction r
                    r.getSources().stream().anyMatch(s -> {
                        return s.getVariable() == portRef.getVariable();
                    }));
                }).collect(Collectors.toList());

        if (reactionsWithPort.isEmpty() && connectionsWithPort.isEmpty()) {
            // Nothing to do here
            return;
        } else {
            // Only add the port for the top-level federate.
            if (isTopLevel) {
                // Add the port to network input ports
                instance.networkInputPorts.add((Input) portRef.getVariable());
            }
        }

        for (Connection connection : connectionsWithPort) {
            // Add the network input control reaction to all the contained
            // reactors, if appropriate
            for (VarRef port : connection.getRightPorts()) {
                addNetworkInputControlReaction(port, recevingPortID, reactor, instance, generator, false);
            }
        }        

        if (!reactionsWithPort.isEmpty()) {
            // If there are reactions at this level, insert the
            // network input control reaction just before them

            String triggerName = "inputControlReactionTriggerFor"
                    + newPortRef.getVariable().getName();

            // Avoid duplicate reactions
            if (reactor.getInputs().stream().anyMatch(i -> {
                return i.getName().equals(triggerName);
            })) {
                return;
            }

            // Find a list of STP offsets (if any exists)
            Set<Value> STPList = CollectionLiterals.<Value>newLinkedHashSet();
            if (generator.isFederatedAndDecentralized()) {
                for (Reaction r : safe(reactionsWithPort)) {
                    // If STP offset is determined, add it
                    // If not, assume it is zero
                    if (r.getStp() != null) {
                        if (r.getStp().getOffset() != null) {
                            STPList.add(r.getStp().getOffset());
                        }
                    }
                }
            }

            // The trigger for the reaction
            VarRef newTriggerForControlReaction = (VarRef) factory
                    .createVarRef();
            Variable newTriggerForControlReactionVariable = factory
                    .createVariable();
            newTriggerForControlReactionVariable.setName(triggerName);
            newTriggerForControlReaction
                    .setVariable(newTriggerForControlReactionVariable);

            reaction.getTriggers().add(newTriggerForControlReaction);
            reaction.getTriggers().add(newPortRef);
            reaction.setCode(factory.createCode());

            reaction.getCode()
                    .setBody(generator.generateNetworkInputControlReactionBody(
                            recevingPortID, STPList));

            // If there are no top-level reactions in this federate that has
            // this
            // port
            // as its trigger or source, there must be a connection to a
            // contained
            // reactor
            // We still take care of the network dependency at this level by
            // injecting
            // a reaction even if there are no other reactions
            int firstIndex = 0;
            // Find the index of the first reaction that has portRef as its
            // trigger or source
            firstIndex = org.icyphy.ASTUtils.allReactions(reactor)
                    .indexOf(reactionsWithPort.get(0));

            // Insert the newly generated reaction before the first reaction
            /// that has the port as its trigger or source
            reactor.getReactions().add(firstIndex, reaction);

            // Create a new Input port for the reaction trigger
            Input newTriggerForControlReactionInput = factory.createInput();
            newTriggerForControlReactionInput
                    .setName(newTriggerForControlReactionVariable.getName());
            Type portType = factory.createType();
            portType.setId(generator.getTargetTimeType());
            newTriggerForControlReactionInput.setType(portType);
            reactor.getInputs().add(newTriggerForControlReactionInput);
        }
    }

    /**
     * Add a network control reaction for a given output port "portRef" to the
     * reactions of the container reactor. This reaction will send a port absent
     * message if the status of the output port is absent.
     * 
     * @note Used in federated execution
     * 
     * @input portRef The output port
     * @input receivingPortID The ID of the receiving port
     * @input instance The federate instance is used to keep track of all
     *        network input ports globally
     * @input generator The GeneratorBase instance used to identify certain
     *        target properties
     */
    public static void addNetworkOutputControlReaction(VarRef portRef,
            FederateInstance instance, int portID, int receivingFedID,
            GeneratorBase generator) {
        LinguaFrancaFactory factory = LinguaFrancaFactory.eINSTANCE;
        Reaction reaction = factory.createReaction();
        Reactor reactor = (Reactor) portRef.eContainer().eContainer();
        // Output
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

        reaction.getTriggers()
                .add(instance.networkOutputControlReactionsTrigger);
        reaction.getTriggers().add(newPortRef);
        reaction.setCode(factory.createCode());

        reaction.getCode().setBody(
                generator.generateNetworkOutputControlReactionBody(portRef,
                        portID, receivingFedID));

        // Insert the newly generated reaction after the generated sender and
        // receiver
        // top-level reactions
        reactor.getReactions().add(reaction);
    }

}
