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
import java.util.Set;
import java.util.stream.Collectors;

import org.eclipse.xtext.xbase.lib.CollectionLiterals;
import org.icyphy.generator.FederateInstance;
import org.icyphy.generator.GeneratorBase;
import org.icyphy.linguaFranca.Connection;
import org.icyphy.linguaFranca.Input;
import org.icyphy.linguaFranca.LinguaFrancaFactory;
import org.icyphy.linguaFranca.Output;
import org.icyphy.linguaFranca.Port;
import org.icyphy.linguaFranca.Reaction;
import org.icyphy.linguaFranca.Reactor;
import org.icyphy.linguaFranca.Value;
import org.icyphy.linguaFranca.VarRef;

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
     *        network input ports globally
     * @input generator The GeneratorBase instance used to indentify certain
     *        target properties
     */
    public static void addNetworkInputControlReaction(VarRef portRef,
            FederateInstance instance, GeneratorBase generator) {
        LinguaFrancaFactory factory = LinguaFrancaFactory.eINSTANCE;
        Reaction reaction = factory.createReaction();
        Reactor reactor = (Reactor) portRef.getVariable().eContainer();
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
        }

        // Add the port to network input ports
        instance.networkInputPorts.add((Input) portRef.getVariable());

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

            // Check the children for STPs as well
            for (Connection c : safe(connectionsWithPort)) {
                VarRef childPort = c.getRightPorts().get(0);
                Reactor childReactor = (Reactor) childPort.getVariable()
                        .eContainer();
                // Find the list of reactions that have the port as trigger or
                // source (could be a variable name)
                List<Reaction> childReactionsWithPort = org.icyphy.ASTUtils
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
                    // If STP offset is determined, add it
                    // If not, assume it is zero
                    if (r.getStp() != null) {
                        if (r.getStp().getOffset() != null) {
                            STPList.add(r.getStp().getOffset());
                        }
                    }
                }
            }
        }

        // FIXME: Ideally, we would like to add this port to the
        // sources and not the triggers to avoid cluttering
        // the trigger table, but a reaction with no triggers
        // will have all the sources as triggers.
        reaction.getTriggers().add(newPortRef);
        reaction.setCode(factory.createCode());

        reaction.getCode()
                .setBody(generator.generateNetworkInputControlReactionBody(
                        (Port) portRef.getVariable(), STPList));

        // If there are no top-level reactions in this federate that has this
        // port
        // as its trigger or source, there must be a connection to a contained
        // reactor
        // We still take care of the network dependency at this level by
        // injecting
        // a reaction even if there are no other reactions
        int firstIndex = 0;
        if (!reactionsWithPort.isEmpty()) {
            // Find the index of the first reaction that has portRef as its
            // trigger or source
            firstIndex = org.icyphy.ASTUtils.allReactions(reactor)
                    .indexOf(reactionsWithPort.get(0));
        }

        // Insert the newly generated reaction before the first reaction
        /// that has the port as its trigger or source
        reactor.getReactions().add(firstIndex, reaction);
    }
    
    /**
     * Add a network control reaction for a given output port "portRef" to the
     * reaction queue of its containing reactor. This reaction will send a port
     * status message if the status of the output port is absent.
     * 
     * @note Used in federated execution
     * 
     * @input portRef The output port
     * @input generator The GeneratorBase instance used to indentify certain
     *        target properties
     */
    public static void addNetworkOutputControlReaction(VarRef portRef,
            GeneratorBase generator) {
        LinguaFrancaFactory factory = LinguaFrancaFactory.eINSTANCE;
        Reaction reaction = factory.createReaction();
        Reactor reactor = (Reactor) portRef.getVariable().eContainer();
        VarRef newPortRef = factory.createVarRef();

        newPortRef.setContainer(null);
        newPortRef.setVariable((Output) portRef.getVariable());

        // First, check if there are any connections to contained reactors that
        // need to be handled
        List<Connection> connectionsWithPort = org.icyphy.ASTUtils
                .allConnections(reactor).stream().filter(c -> {
                    return c.getRightPorts().stream().anyMatch((VarRef v) -> {
                        return v.getVariable().equals(portRef.getVariable());
                    });
                }).collect(Collectors.toList());

        // Find the list of reactions that have the port as trigger or source
        // (could be a variable name)
        List<Reaction> reactionsWithPort = org.icyphy.ASTUtils
                .allReactions(reactor).stream().filter(r -> {
                    // Check the triggers of reaction r first
                    return r.getEffects().stream().anyMatch(t -> {
                        if (t instanceof Output) {
                            // Check if the variables match
                            return ((VarRef) t).getVariable() == portRef
                                    .getVariable();
                        } else {
                            // Inputs to contained reactors
                            return false;
                        }
                    });
                }).collect(Collectors.toList());

        if (reactionsWithPort.isEmpty() && connectionsWithPort.isEmpty()) {
            // Nothing to do here
            return;
        }
        reaction.getEffects().add(newPortRef);
        reaction.setCode(factory.createCode());

        reaction.getCode()
                .setBody(generator.generateNetworkOutputControlReactionBody(
                        (Port) portRef.getVariable()));

        // If there are no top-level reactions in this federate that has this
        // port
        // as its trigger or source, there must be a connection to a contained
        // reactor
        // We still take care of the network dependency at this level by
        // injecting
        // a reaction even if there are no other reactions
        int firstIndex = 0;
        if (!reactionsWithPort.isEmpty()) {
            // Find the index of the first reaction that has portRef as its
            // trigger or source
            firstIndex = org.icyphy.ASTUtils.allReactions(reactor)
                    .indexOf(reactionsWithPort.get(0));
        }

        // Insert the newly generated reaction before the first reaction
        /// that has the port as its trigger or source
        reactor.getReactions().add(firstIndex, reaction);
    }

}
