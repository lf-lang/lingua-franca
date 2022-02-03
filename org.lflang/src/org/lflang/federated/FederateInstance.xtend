/** Instance of a federate specification. */

/*************
Copyright (c) 2020, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

package org.lflang.federated

import java.util.ArrayList
import java.util.LinkedHashMap
import java.util.LinkedHashSet
import java.util.List
import java.util.Set
import org.lflang.ErrorReporter
import org.lflang.TimeValue
import org.lflang.generator.ActionInstance
import org.lflang.generator.GeneratorBase
import org.lflang.generator.PortInstance
import org.lflang.generator.ReactionInstance
import org.lflang.generator.ReactorInstance
import org.lflang.lf.Action
import org.lflang.lf.ActionOrigin
import org.lflang.lf.Delay
import org.lflang.lf.Input
import org.lflang.lf.Instantiation
import org.lflang.lf.Mode
import org.lflang.lf.Output
import org.lflang.lf.Port
import org.lflang.lf.Reaction
import org.lflang.lf.Reactor
import org.lflang.lf.Timer
import org.lflang.lf.TriggerRef
import org.lflang.lf.VarRef
import org.lflang.lf.Variable

import static extension org.lflang.ASTUtils.*
import static extension org.lflang.ModesUtil.*
import org.lflang.generator.GeneratorBase
import org.lflang.generator.ReactorInstance
import org.lflang.generator.ReactionInstance
import org.lflang.generator.ActionInstance
import org.lflang.generator.PortInstance

/** 
 * Instance of a federate, or marker that no federation has been defined
 * (if isSingleton() returns true). Every top-level reactor (contained
 * directly by the main reactor) is a federate, so there will be one
 * instance of this class for each top-level reactor.
 * 
 * @author{Edward A. Lee <eal@berkeley.edu>}
 */
class FederateInstance {

    /**
     * Construct a new instance with the specified instantiation of
     * of a top-level reactor. The federate will be given the specified
     * integer ID.
     * @param instantiation The instantiation of a top-level reactor,
     *  or null if no federation has been defined.
     * @param id The federate ID.
     * @param bankIndex If instantiation.widthSpec !== null, this gives the bank position.
     * @param generator The generator
     * @param errorReporter The error reporter
     * 
     * FIXME: Do we really need to pass the complete generator here? It is only used 
     *  to determine the number of federates.
     */
    new(
        Instantiation instantiation, 
        int id, 
        int bankIndex, 
        GeneratorBase generator,
        ErrorReporter errorReporter
    ) {
        this.instantiation = instantiation;
        this.id = id;
        this.generator = generator;
        this.bankIndex = bankIndex;
        this.errorReporter = errorReporter;
                
        if (instantiation !== null) {
            this.name = instantiation.name;
            // If the instantiation is in a bank, then we have to append
            // the bank index to the name.
            if (instantiation.widthSpec !== null) {
                this.name = instantiation.name + "__" + bankIndex;
            }
        }
    }

    /////////////////////////////////////////////
    //// Public Fields
    
    /**
     * The position within a bank of reactors for this federate.
     * This is 0 if the instantiation is not a bank of reactors.
     */
    public var bankIndex = 0;
    
    /**
     * A list of outputs that can be triggered directly or indirectly by physical actions.
     */
    public var outputsConnectedToPhysicalActions = new LinkedHashSet<Delay>()
    
    /** The host, if specified using the 'at' keyword. */
    public var String host = 'localhost'
    
    /** The instantiation of the top-level reactor, or null if there is no federation. */
    public var Instantiation instantiation;
    
    /**
     * Map from the federates that this federate receives messages from
     * to the delays on connections from that federate. The delay set
     * may may include null, meaning that there is a connection
     * from the federate instance that has no delay.
     */
    public var dependsOn = new LinkedHashMap<FederateInstance,Set<Delay>>()
    
    /** The directory, if specified using the 'at' keyword. */
    public var String dir = null

    /** The port, if specified using the 'at' keyword. */
    public var int port = 0
    
    /** 
     * Map from the federates that this federate sends messages to
     * to the delays on connections to that federate. The delay set
     * may may include null, meaning that there is a connection
     * from the federate instance that has no delay.
     */
    public var sendsTo = new LinkedHashMap<FederateInstance,Set<Delay>>()
        
    /** The user, if specified using the 'at' keyword. */
    public var String user = null
    
    /** The integer ID of this federate. */
    public var id = 0;
    
    /**
     * The name of this federate instance. This will be the instantiation
     * name, poassibly appended with "__n", where n is the bank position of
     * this instance if the instantiation is of a bank of reactors.
     */
    public var name = "Unnamed";
    
    /** List of networkMessage actions. Each of these handles a message
     *  received from another federate over the network. The ID of
     *  receiving port is simply the position of the action in the list.
     *  The sending federate needs to specify this ID.
     */
    public var List<Action> networkMessageActions = new ArrayList<Action>()
    
    /** 
     * A set of federates with which this federate has an inbound connection
     * There will only be one physical connection even if federate A has defined multiple
     * physical connections to federate B. The message handler on federate A will be 
     * responsible for including the appropriate information in the message header (such as port ID)
     * to help the receiver distinguish different events.
     */
    public var inboundP2PConnections = new LinkedHashSet<FederateInstance>()
    
    /**
     * A list of federate with which this federate has an outbound physical connection. 
     * There will only be one physical connection even if federate A has defined multiple
     * physical connections to federate B. The message handler on federate B will be 
     * responsible for distinguishing the incoming messages by parsing their header and
     * scheduling the appropriate action.
     */
    public var outboundP2PConnections = new LinkedHashSet<FederateInstance>()
    
        
    /**
     * A list of triggers for network input control reactions. This is used to trigger
     * all the input network control reactions that might be nested in a hierarchy.
     */
    public var List<Port> networkInputControlReactionsTriggers = new ArrayList<Port>();
    
    
    /**
     * The trigger that triggers the output control reaction of this 
     * federate. 
     * 
     * The network output control reactions send a PORT_ABSENT message for a network output port, 
     * if it is absent at the current tag, to notify all downstream federates that no value will 
     * be present on the given network port, allowing input control reactions on those federates 
     * to stop blocking.
     */
    public var Variable networkOutputControlReactionsTrigger = null;
    
    /**
     * Indicates whether the federate is remote or local
     */
    public var boolean isRemote = false;
    
    
    /**
     * List of generated network reactions (network receivers,
     * network input control reactions, network senders, and network output control
     * reactions) that belong to this federate instance.
     */
     public List<Reaction> networkReactions = new ArrayList<Reaction>();
     
     /**
      * List of triggers of network reactions that belong to remote federates.
      * These might need to be removed before code generation to avoid unnecessary compile
      * errors, since they might reference structures that are not present in
      * the current federate. Even though it is impossible for a trigger that is on a remote
      * federate to trigger a reaction on this federate, these triggers need to be here
      * to ensure that dependency analysis between reactions is done correctly.
      * Without these triggers, the reaction precedence graph is broken and
      * dependencies not properly represented.
      */
     public List<VarRef> remoteNetworkReactionTriggers = new ArrayList<VarRef>();

    /////////////////////////////////////////////
    //// Public Methods

    /**
     * Return true if the specified action should be included in the code generated
     * for the federate. This means that either the action is used as a trigger,
     * a source, or an effect in a top-level reaction that belongs to this federate.
     * This returns true if the program is not federated.
     * 
     * @param action The action
     * @return True if this federate contains the action in the specified reactor
     */
    def contains(Action action) {
        val reactor  = getEnclosingReactor(action)
        if (!reactor.federated || isSingleton) return true
        
        // If the action is used as a trigger, a source, or an effect for a top-level reaction
        // that belongs to this federate, then generate it.
        for (react : reactor.allReactions) {
            if (contains(react)) {
                // Look in triggers
                for (TriggerRef trigger : react.triggers ?: emptyList) {
                    if (trigger instanceof VarRef) {
                        if (trigger.variable == (action as Variable)) {
                            return true;
                        }
                    }
                }
                // Look in sources
                for (VarRef source : react.sources ?: emptyList) {
                    if (source.variable == (action as Variable)) {
                        return true;
                    }
                }
                // Look in effects
                for (effect : react.effects ?: emptyList) {
                    if (effect.variable == (action as Variable)) {
                        return true;
                    }
                }
            }
        }
        
        return false;        
    }
    
    /**
     * Return true if the specified reactor is not the top-level federated reactor,
     * or if it is and the port should be included in the code generated
     * for the federate. This means that the port has been used as a trigger, 
     * a source, or an effect in a top-level reaction that belongs to this federate.
     * 
     * @param port The Port
     * @return True if this federate contains the action in the specified reactor
     */
    def contains(Port port) {
        val reactor  = port.eContainer as Reactor
        if (!reactor.federated || isSingleton) return true
        
        // If the port is used as a trigger, a source, or an effect for a top-level reaction
        // that belongs to this federate, then return true.
        for (react : reactor.allReactions) {
            if (contains(react)) {
                // Look in triggers
                for (TriggerRef trigger : react.triggers ?: emptyList) {
                    if (trigger instanceof VarRef) {
                        if (trigger.variable == (port as Variable)) {
                            return true;
                        }
                    }
                }
                // Look in sources
                for (VarRef source : react.sources ?: emptyList) {
                    if (source.variable == (port as Variable)) {
                        return true;
                    }
                }
                // Look in effects
                for (effect : react.effects ?: emptyList) {
                    if (effect.variable == (port as Variable)) {
                        return true;
                    }
                }
            }
        }
        
        return false;
    }
        
    /** 
     * Return true if the specified reaction should be included in the code generated for this
     * federate at the top-level. This means that if the reaction is triggered by or
     * sends data to a port of a contained reactor, then that reaction
     * is in the federate. Otherwise, return false.
     * 
     * As a convenience measure, also return true if the reaction is not defined in the top-level 
     * (federated) reactor, or if the top-level reactor is not federated.
     *
     * @param reaction The reaction.
     */
    def contains(Reaction reaction) {
        val reactor  = getEnclosingReactor(reaction)
        // Easy case first.
        if (!reactor.federated || isSingleton) return true
        
        if (!reactor.reactions.contains(reaction)) return false;
        
        if (networkReactions.contains(reaction)) {
            // Reaction is a network reaction that belongs to this federate
            return true;
        }
        
        val reactionBankIndex = generator.getReactionBankIndex(reaction)
        if (reactionBankIndex >= 0 && this.bankIndex >= 0 && reactionBankIndex != this.bankIndex) {
            return false;
        }
        
        // If this has been called before, then the result of the
        // following check is cached.
        if (excludeReactions !== null) {
            return !excludeReactions.contains(reaction)
        }
        
        indexExcludedTopLevelReactions(reactor);
       
        return !excludeReactions.contains(reaction)
    }
    
    /** 
     * Return true if the specified reactor instance or any parent
     * reactor instance is contained by this federate.
     * If the specified instance is the top-level reactor, return true
     * (the top-level reactor belongs to all federates).
     * If this federate instance is a singleton, then return true if the
     * instance is non null.
     * 
     * NOTE: If the instance is bank within the top level, then this
     * returns true even though only one of the bank members is in the federate.
     * 
     * @param instance The reactor instance.
     * @return True if this federate contains the reactor instance
     */
    def contains(ReactorInstance instance) {
        if (isSingleton) {
            return (instance !== null);
        }
        if (instance.parent === null) {
            return true; // Top-level reactor
        }
        // Start with this instance, then check its parents.
        var i = instance;
        while (i !== null) {
            if (i.definition === this.instantiation) {
                return true;
            }
            i = i.parent;
        }
        return false;
    }
    
    /**
     * Return true if the specified timer should be included in the code generated
     * for the federate. This means that the timer is used as a trigger
     * in a top-level reaction that belongs to this federate.
     * This also returns true if the program is not federated.
     * 
     * @param action The action
     * @return True if this federate contains the action in the specified reactor
     */
    def contains(Timer timer) {
        val reactor  = timer.eContainer as Reactor
        if (!reactor.federated || isSingleton) return true
        
        // If the action is used as a trigger, a source, or an effect for a top-level reaction
        // that belongs to this federate, then generate it.
        for (r : reactor.allReactions) {
            if (contains(r)) {
                // Look in triggers
                for (TriggerRef trigger : r.triggers ?: emptyList) {
                    if (trigger instanceof VarRef) {
                        if (trigger.variable == (timer as Variable)) {
                            return true;
                        }
                    }
                }
            }
        }
        
        return false;        
    }
    
    /**
     * Return the total number of runtime instances of the specified reactor
     * instance in this federate. This is zero if the reactor is not in the
     * federate at all, and otherwise is the product of the bank widths of
     * all the parent containers of the instance, except that if the depth
     * one parent is bank, its width is ignored (only one bank member can be
     * in any federate).
     */
    def numRuntimeInstances(ReactorInstance reactor) {
        if (!contains(reactor)) return 0;
        val depth = isSingleton ? 0 : 1;
        return reactor.getTotalWidth(depth);
    }

    /**
     * Build an index of reactions at the top-level (in the
     * federatedReactor) that don't belong to this federate
     * instance. This index is put in the excludeReactions
     * class variable.
     * 
     * @param federatedReactor The top-level federated reactor
     */
    private def indexExcludedTopLevelReactions(Reactor federatedReactor) {
        var inFederate = false
        if (excludeReactions !== null) {
            throw new IllegalStateException("The index for excluded reactions at the top level is already built.")
        }

        excludeReactions = new LinkedHashSet<Reaction>

        // Construct the set of excluded reactions for this federate.
        // If a reaction is a network reaction that belongs to this federate, we
        // don't need to perform this analysis.
        for (react : federatedReactor.allReactions.filter[reaction|!networkReactions.contains(reaction)]) {
            // Create a collection of all the VarRefs (i.e., triggers, sources, and effects) in the react 
            // signature that are ports that reference federates.
            // We then later check that all these VarRefs reference this federate. If not, we will add this
            // react to the list of reactions that have to be excluded (note that mixing VarRefs from
            // different federates is not allowed).
            var allVarRefsReferencingFederates = new ArrayList<VarRef>();
            // Add all the triggers that are outputs
            allVarRefsReferencingFederates.addAll(
                react.triggers.filter[it instanceof VarRef].map[it as VarRef].filter[it.variable instanceof Output].toList
            )
            // Add all the sources that are outputs
            allVarRefsReferencingFederates.addAll(
                react.sources.filter[it.variable instanceof Output].toList
            )
            // Add all the effects that are inputs
            allVarRefsReferencingFederates.addAll(
                react.effects.filter[it.variable instanceof Input].toList
            );
            inFederate = containsAllVarRefs(allVarRefsReferencingFederates)
            if (!inFederate) {
                excludeReactions.add(react)
            }
        }
    }
    
    /**
     * Return true if all members of 'varRefs' belong to this federate.
     * 
     * As a convenience measure, if some members of 'varRefs' are from 
     * different federates, also report an error.
     * 
     * @param varRefs A collection of VarRefs
     */
    private def containsAllVarRefs(Iterable<VarRef> varRefs) {
        var referencesFederate = false;
        var inFederate = true;
        for (varRef : varRefs) {
            if (varRef.container === this.instantiation) {
                referencesFederate = true;
            } else {
                if (referencesFederate) {
                    errorReporter.reportError(varRef, "Mixed triggers and effects from" +
                        " different federates. This is not permitted")
                }
                inFederate = false;
            }
        }
        return inFederate;
    }
    
    /** 
     * Return true if this is singleton, meaning either that no federation
     * has been defined or that there is only one federate.
     * @return True if no federation has been defined or there is only one federate.
     */
    def isSingleton() {
        return ((instantiation === null) || (generator.federates.size <= 1))
    }
     
    /**
     * Find output ports that are connected to a physical action trigger upstream
     * in the same reactor. Return a list of such outputs paired with the minimum delay
     * from the nearest physical action.
     * @param instance The reactor instance containing the output ports
     * @return A LinkedHashMap<Output, TimeValue>
     */
    def findOutputsConnectedToPhysicalActions(ReactorInstance instance) {
        var physicalActionToOutputMinDelay = new LinkedHashMap<Output, TimeValue>()
        // Find reactions that write to the output port of the reactor
        for (output : instance.outputs) {
            for (reaction : output.dependsOnReactions) {
                var minDelay = findNearestPhysicalActionTrigger(reaction)
                if (minDelay != TimeValue.MAX_VALUE) {
                    physicalActionToOutputMinDelay.put(output.definition as Output, minDelay)
                }
            }
        }
        return physicalActionToOutputMinDelay
    }
    
    override toString() {
        "Federate " + this.id + ": " + instantiation.name
    }

    /////////////////////////////////////////////
    //// Private Fields
     
    /** Cached result of analysis of which reactions to exclude from main. */
    var excludeReactions = null as Set<Reaction>
    
    /** The generator using this. */
    var generator = null as GeneratorBase
    
    /** Returns the generator that is using this federate instance */
    def getGenerator() { return generator; }
    
    /** An error reporter */
    val ErrorReporter errorReporter
    
    /**
     * Find the nearest (shortest) path to a physical action trigger from this
     * 'reaction' in terms of minimum delay.
     * 
     * @param reaction The reaction to start with
     * @return The minimum delay found to the nearest physical action and
     *  TimeValue.MAX_VALUE otherwise
     */
    def TimeValue findNearestPhysicalActionTrigger(ReactionInstance reaction) {
        var minDelay = TimeValue.MAX_VALUE;
        for (trigger : reaction.triggers) {
            if (trigger.definition instanceof Action) {
                var action = trigger.definition as Action
                var actionInstance = trigger as ActionInstance
                if (action.origin === ActionOrigin.PHYSICAL) {
                    if (actionInstance.minDelay.isEarlierThan(minDelay)) {
                        minDelay = actionInstance.minDelay;
                    }
                } else if (action.origin === ActionOrigin.LOGICAL) {
                    // Logical action
                    // Follow it upstream inside the reactor
                    for (uReaction: actionInstance.dependsOnReactions) {
                        // Avoid a loop
                        if (uReaction != reaction) {
                            var uMinDelay = actionInstance.minDelay.add(findNearestPhysicalActionTrigger(uReaction))
                            if (uMinDelay.isEarlierThan(minDelay)) {
                                minDelay = uMinDelay;
                            }
                        }
                    }
                }
                
            } else if (trigger.definition instanceof Output) {
                // Outputs of contained reactions
                var outputInstance = trigger as PortInstance
                for (uReaction: outputInstance.dependsOnReactions) {
                    var uMinDelay = findNearestPhysicalActionTrigger(uReaction)
                    if (uMinDelay.isEarlierThan(minDelay)) {
                        minDelay = uMinDelay;
                    }
                }
            }
        }
        return minDelay
    }
    
    /**
     * Remove triggers in this federate's network reactions that are defined in remote federates.
     */
    def removeRemoteFederateConnectionPorts() {
        for (reaction: networkReactions) {
            reaction.getTriggers().removeAll(remoteNetworkReactionTriggers)
        }
    }
    
}
