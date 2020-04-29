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

package org.icyphy.generator

import java.util.HashMap
import java.util.HashSet
import java.util.LinkedList
import java.util.Set
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.KeyValuePair
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Value
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.TriggerRef
import org.icyphy.linguaFranca.VarRef

/** Instance of a federate, or marker that no federation has been defined
 *  (if isSingleton() returns true).
 * 
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 */
class FederateInstance {
    

    /** Construct a new instance with the specified definition
     *  and parent. The definition has a name (the federate name)
     *  and a value (of type Element), where the value has a field
     *  keyvalue (of type KeyValuePair) containing the properties
     *  of the federate.  For convenient access, those properties
     *  are exported by getting methods of this instance.
     *  @param definition The definition in the AST for this
     *   instance, or null if no federation has been defined.
     *  @param The generator (for reporting errors).
     */
    protected new(KeyValuePair definition, int id, GeneratorBase generator) {
        this.definition = definition
        this.id = id
        this.generator = generator
                
        // Populate the contained reactor names.
        if (definition !== null) {
            // NOTE: Validator checks for the following structure.
            for (property : definition.value.keyvalue.pairs) {
                if (property.name.equals("reactors")) {
                    for (reactor : property.value.array.elements) {
                        containedReactorNames.add(reactor.id)
                    }
                }
            }
        }
    }

    /////////////////////////////////////////////
    //// Public Fields
    
    /** Set of names of contained reactors. Note that will be
     *  empty if isSingleton() returns true.
     */
    public var Set<String> containedReactorNames = new HashSet<String>
    
    /** The Instantiation AST object from which this was created. */
    public var KeyValuePair definition
    
    /** Map from the federates that this federate receives messages from
     *  to the delays on connections from that federate. The delay set
     *  may be empty, meaning no delay (not even a microstep or 0 delay)
     *  was specified.
     */
    public var dependsOn = new HashMap<FederateInstance,Set<Value>>()
    
    /** Map from the federates that this federate sends messages to
     *  to the delays on connections to that federate. The delay set
     *  may be empty, meaning no delay (not even a microstep or 0 delay)
     *  was specified.
     */
    public var sendsTo = new HashMap<FederateInstance,Set<Value>>()
    
    /** The integer ID of this federate. */
    public var id = 0;
    
    /** List of networkMessage actions. Each of these handles a message
     *  received from another federate over the network. The ID of
     *  receiving port is simply the position of the action in the list.
     *  The sending federate needs to specify this ID.
     */
    public var networkMessageActions = new LinkedList<Action>()

    /////////////////////////////////////////////
    //// Public Methods
    
    /** Return true if the specified reactor name is contained by
     *  this federate.
     *  @return True if the federate contains the reactor.
     */
    def contains(String reactorName) {
        containedReactorNames.contains(reactorName) 
    }
        
    /** Return true if the specified reactor is not the main reactor,
     *  or if it is and the reaction should be included in the code generated for the
     *  federate. This means that if the reaction is triggered by or
     *  sends data to a port of a contained reactor, then that reactor
     *  is in the federate. Otherwise, return false.
     *  @param reaction The reaction.
     *  @param federate The federate instance or null if there
     *   is no federation.
     */
    def containsReaction(Reactor reactor, Reaction reaction) {
        // Easy case first.
        if (!reactor.federated || isSingleton) return true
        
        // If this has been called before, then the result of the
        // following check is cached.
        if (excludeReactions !== null) {
            return !excludeReactions.contains(reaction)
        }
        excludeReactions = new HashSet<Reaction>
        
        // Construct the set of excluded reactions for this federate.
        for (react : reactor.reactions) {
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
                        if (contains(trigger.container.name)) {
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
                    if (contains(effect.container.name)) {
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
    
    /** Return the name of this federate. 
     *  @return The name of this federate.
     */
    def getName() {
        this.definition.name
    }
    
    /** Return true if this is singleton, meaning that no federation
     *  has been defined.
     *  @return True if no federation has been defined.
     */
     def isSingleton() {
         return (definition === null)
     }

    /////////////////////////////////////////////
    //// Private Fields
     
    /** Cached result of analysis of which reactions to exclude from main. */
    var excludeReactions = null as Set<Reaction>
    
    /** The generator using this. */
    var generator = null as GeneratorBase 
}
