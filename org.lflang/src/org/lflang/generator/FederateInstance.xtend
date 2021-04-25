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

package org.lflang.generator

import java.util.LinkedHashMap
import java.util.LinkedHashSet
import java.util.LinkedList
import java.util.Set
import org.lflang.TimeValue
import org.lflang.lf.Action
import org.lflang.lf.ActionOrigin
import org.lflang.lf.Delay
import org.lflang.lf.Input
import org.lflang.lf.Instantiation
import org.lflang.lf.Output
import org.lflang.lf.Reaction
import org.lflang.lf.Reactor
import org.lflang.lf.TriggerRef
import org.lflang.lf.VarRef

import static extension org.lflang.ASTUtils.*

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
     * @param generator The generator (for reporting errors).
     */
    protected new(Instantiation instantiation, int id, int bankIndex, GeneratorBase generator) {
        this.instantiation = instantiation;
        this.id = id;
        this.generator = generator;
        this.bankIndex = bankIndex;
                
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
     * may be empty, meaning no delay (not even a microstep or 0 delay)
     * was specified.
     */
    public var dependsOn = new LinkedHashMap<FederateInstance,Set<Delay>>()
    
    /** The directory, if specified using the 'at' keyword. */
    public var String dir = null

    /** The port, if specified using the 'at' keyword. */
    public var int port = 0
    
    /** Map from the federates that this federate sends messages to
     *  to the delays on connections to that federate. The delay set
     *  may be empty, meaning no delay (not even a microstep or 0 delay)
     *  was specified.
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
    public var networkMessageActions = new LinkedList<Action>()
    
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

    /////////////////////////////////////////////
    //// Public Methods
    
    /** 
     * Return true if the specified reactor instance or any parent
     * reactor instance is contained by this federate.
     * If the specified instance is the top-level reactor, return true
     * (this reactor belongs to all federates).
     * If it is a bank member, then this returns true only if the bankIndex
     * of the reactor instance matches the federate instance bank index.
     * If this federate instance is a singleton, then return true if the
     * instance is non null.
     * 
     * @param instance The reactor instance.
     * @return True if this federate contains the reactor instance
     */
    def contains(ReactorInstance instance) {
        if (isSingleton) {
            return (instance !== null);
        }
        if (instance.parent === null) {
            return true;
        }
        // Start with this instance, then check its parents.
        var i = instance;
        while (i !== null) {
            if (i.definition === this.instantiation
                    && (
                        i.bankIndex < 0                 // Not a bank member
                        || i.bankIndex == this.bankIndex  // Index matches.
                    )
            ) {
                return true;
            }
            i = i.parent;
        }
        return false;
    }
        
    /** 
     * Return true if the specified reactor is not the main reactor,
     * or if it is and the reaction should be included in the code generated for the
     * federate. This means that if the reaction is triggered by or
     * sends data to a port of a contained reactor, then that reactor
     * is in the federate. Otherwise, return false.
     * @param reaction The reaction.
     * @param federate The federate instance or null if there
     *  is no federation.
     */
    def containsReaction(Reactor reactor, Reaction reaction) {
        // Easy case first.
        if (!reactor.federated || isSingleton) return true
        
        // If this has been called before, then the result of the
        // following check is cached.
        if (excludeReactions !== null) {
            return !excludeReactions.contains(reaction)
        }
        excludeReactions = new LinkedHashSet<Reaction>
        
        // Construct the set of excluded reactions for this federate.
        for (react : reactor.allReactions) {
            // If the reaction is triggered by an output of a contained
            // reactor that is not in the federate, or the reaction
            // sends to an input of a contained reactor that is not
            // in the federate, then do not generate code for the reaction.
            // If the reaction mixes ports across federates, then report
            // an error and do not generate code.
            var referencesFederate = false;
            var inFederate = true;
            for (TriggerRef trigger : react.triggers ?: emptyList) {
                if (trigger instanceof VarRef) {
                    if (trigger.variable instanceof Output) {
                        // The trigger is an output port of a contained reactor.
                        if (trigger.container === this.instantiation) {
                            referencesFederate = true;
                        } else {
                            if (referencesFederate) {
                                generator.reportError(react, 
                                "Reaction mixes triggers and effects from" +
                                " different federates. This is not permitted")
                            }
                            inFederate = false;
                        }
                    }
                }
            }
            for (effect : react.effects ?: emptyList) {
                if (effect.variable instanceof Input) {
                    // It is the input of a contained reactor.
                    if (effect.container === this.instantiation) {
                        referencesFederate = true;
                    } else {
                        if (referencesFederate) {
                            generator.reportError(react,
                                "Reaction mixes triggers and effects from" + 
                                " different federates. This is not permitted")
                        }
                        inFederate = false;
                    }
                }
            }
            if (!inFederate) {
                excludeReactions.add(react)
            }
        }
        return !excludeReactions.contains(reaction)
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

    /////////////////////////////////////////////
    //// Private Fields
     
    /** Cached result of analysis of which reactions to exclude from main. */
    var excludeReactions = null as Set<Reaction>
    
    /** The generator using this. */
    var generator = null as GeneratorBase
    
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
                } else {
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
                    println("Found reaction " + uReaction.name + " upstream in contained reactors")
                    var uMinDelay = findNearestPhysicalActionTrigger(uReaction)
                    if (uMinDelay.isEarlierThan(minDelay)) {
                        minDelay = uMinDelay;
                    }
                }
            }
        }
        return minDelay
    }
}
