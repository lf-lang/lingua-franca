/** Representation of a runtime instance of a reaction. */

/*************
Copyright (c) 2019, The University of California at Berkeley.

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

import java.util.LinkedHashSet
import org.lflang.TimeValue
import org.lflang.lf.Action
import org.lflang.lf.Port
import org.lflang.lf.Reaction
import org.lflang.lf.TimeUnit
import org.lflang.lf.Timer
import org.lflang.lf.TriggerRef
import org.lflang.lf.VarRef
import org.lflang.lf.Variable

/**
 * Representation of a runtime instance of a reaction.
 * A ReactionInstance object stores all dependency information necessary
 * for constructing a directed acyclic precedece graph.
 *  
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class ReactionInstance extends NamedInstance<Reaction> {

    /**
     * Create a new reaction instance from the specified definition
     * within the specified parent. This constructor should be called
     * only by the ReactionInstance class.
     * @param definition A reaction definition.
     * @param parent The parent reactor instance, which cannot be null.
     * @param index The index of the reaction within the reactor (0 for the
     * first reaction, 1 for the second, etc.).
     */
    protected new(Reaction definition, ReactorInstance parent, boolean isUnordered, int index) {
        super(definition, parent);
        this.reactionIndex = index
        this.isUnordered = isUnordered
        
        // Identify the dependencies for this reaction.
        // First handle the triggers.
        for (TriggerRef trigger : definition.triggers) {
            if (trigger instanceof VarRef) {
                if (trigger.variable instanceof Port) {
                    var portInstance = parent.lookupPortInstance(trigger)
                    // If the trigger is the port of a contained bank, then the
                    // portInstance will be null and we have to instead search for
                    // each port instance in the bank.
                    if (portInstance !== null) {
                        this.sources.add(portInstance)
                        portInstance.dependentReactions.add(this)
                        this.triggers.add(portInstance)
                    } else if (trigger.container !== null) {
                        val bankInstance = parent.lookupReactorInstance(trigger.container)
                        if (bankInstance !== null && bankInstance.bankMembers !== null) {
                            for (bankMember : bankInstance.bankMembers) {
                                portInstance = bankMember.lookupPortInstance(trigger.variable as Port)
                                if (portInstance !== null) {
                                    this.sources.add(portInstance)
                                    portInstance.dependentReactions.add(this)
                                    this.triggers.add(portInstance)
                                }
                            }
                        }
                    }
                } else if (trigger.variable instanceof Action) {
                    var actionInstance = parent.lookupActionInstance(
                        trigger.variable as Action)
                    this.triggers.add(actionInstance)
                    actionInstance.dependentReactions.add(this)
                    this.sources.add(actionInstance)
                } else if (trigger.variable instanceof Timer) {
                    var timerInstance = parent.lookupTimerInstance(
                        trigger.variable as Timer)
                    this.triggers.add(timerInstance)
                    timerInstance.dependentReactions.add(this)
                }
            } else if (trigger.startup) {
                this.triggers.add(parent.getOrCreateStartup(trigger))
            } else if (trigger.shutdown) {
                this.triggers.add(parent.getOrCreateShutdown(trigger))
            }
        }
        // Next handle the ports that this reaction reads.
        for (source : definition.sources) {
            if (source.variable instanceof Port) {
                var portInstance = parent.lookupPortInstance(source)
                // If the trigger is the port of a contained bank, then the
                // portInstance will be null and we have to instead search for
                // each port instance in the bank.
                if (portInstance !== null) {
                    this.sources.add(portInstance)
                    this.reads.add(portInstance)
                    portInstance.dependentReactions.add(this)
                } else if (source.container !== null) {
                    val bankInstance = parent.lookupReactorInstance(source.container)
                    if (bankInstance !== null && bankInstance.bankMembers !== null) {
                        for (bankMember : bankInstance.bankMembers) {
                            portInstance = bankMember.lookupPortInstance(source.variable as Port)
                            if (portInstance !== null) {
                                this.sources.add(portInstance)
                                portInstance.dependentReactions.add(this)
                                this.reads.add(portInstance)
                            }
                        }
                    }
                }
            }
        }

        // Finally, handle the effects.
        for (effect : definition.effects) {
            if (effect.variable instanceof Port) {
                var portInstance = parent.lookupPortInstance(effect)
                if (portInstance instanceof MultiportInstance) {
                    for (multiportInstance : portInstance.instances) {
                        this.effects.add(multiportInstance)
                        multiportInstance.dependsOnReactions.add(this)
                    }
                } else if (portInstance !== null) {
                    this.effects.add(portInstance)
                    portInstance.dependsOnReactions.add(this)
                } else {
                    // The effect container must be a bank of reactors.
                    // Need to find the ports of all the instances within the bank.
                    val bank = parent.lookupReactorInstance(effect.container);
                    if (bank === null || bank.bankIndex != -2) {
                        throw new InvalidSourceException("Unexpected effect. Cannot find port " + effect.variable.name);
                    }
                    for (bankElement : bank.bankMembers) {
                        portInstance = bankElement.lookupPortInstance(effect.variable as Port);
                        if (portInstance instanceof MultiportInstance) {
                            for (multiportInstance : portInstance.instances) {
                                this.effects.add(multiportInstance)
                                multiportInstance.dependsOnReactions.add(this)
                            } 
                        } else if (portInstance === null) {
                            throw new InvalidSourceException("Unexpected effect. Cannot find port within bank: " + effect.variable.name);
                        }
                        this.effects.add(portInstance)
                        portInstance.dependsOnReactions.add(this);
                    }
                }
            } else if (effect.variable instanceof Action) {
                var actionInstance = parent.lookupActionInstance(
                    effect.variable as Action)
                this.effects.add(actionInstance)
                actionInstance.dependsOnReactions.add(this)
            } // else it may be an unresolved reference
        }
        // Create a deadline instance if one has been defined.
        if (this.definition.deadline !== null) {
            this.declaredDeadline = new DeadlineInstance(
                this.definition.deadline, this)
        }
    }
    
    /**
     * Purge 'portInstance' from this reaction
     */
    def removePortInstance(PortInstance portInstance) {
        if (portInstance instanceof MultiportInstance) {
            for (multiportInstance : portInstance.instances) {
                this.triggers.removeAll(portInstance);
                this.sources.removeAll(portInstance);
                this.effects.removeAll(portInstance);
                this.reads.removeAll(portInstance);

            }
        }
        this.triggers.removeAll(portInstance);
        this.sources.removeAll(portInstance);
        this.effects.removeAll(portInstance);
        this.reads.removeAll(portInstance);
    }

    /** 
     * Indicates the chain this reaction is a part of. It is constructed
     * through a bit-wise or among all upstream chains. Each fork in the
     * dependency graph setting a new, unused bit to true in order to
     * disambiguate it from parallel chains.
     */
    public long chainID = 0L;

    /**
     * The ports or actions that this reaction may write to.
     */
    public var effects = new LinkedHashSet<TriggerInstance<? extends Variable>>();

    /**
     * The ports, actions, or timers that this reaction is triggered by or uses.
     */
    public var sources = new LinkedHashSet<TriggerInstance<? extends Variable>>();

    /**
     * Deadline for this reaction instance, if declared.
     */
    public DeadlineInstance declaredDeadline

    /**
     * Inferred deadline. Defaults to the maximum long value.
     */
    public var deadline = new TimeValue(TimeValue.MAX_LONG_DEADLINE, TimeUnit.NSEC)

    /**
     * The level in the dependence graph. -1 indicates that the level
     * has not yet been assigned.
     */
    public long level = -1L;

    /**
     * Index of order of occurrence within the reactor definition.
     * The first reaction has index 0, the second index 1, etc.
     */
    public int reactionIndex = -1;

    /**
     * Whether or not this reaction is ordered with respect to other
     * reactions in the same reactor.
     */
    public boolean isUnordered

    /**
     * The ports that this reaction reads but that do not trigger it.
     */
    public var reads = new LinkedHashSet<TriggerInstance<? extends Variable>>

    /**
     * The trigger instances (input ports, timers, and actions
     * that trigger reactions) that trigger this reaction.
     */
    public var triggers = new LinkedHashSet<TriggerInstance<? extends Variable>>

    /**
     * Sources through which this reaction instance has been visited.
     */
    public var visited = new LinkedHashSet<ReactionInstance>

    /**
     * Counter that indicates how many times this node has been visited during
     * the graph traversal that sets the chainIDs. Only when this counter hits zero
     * shall the traversal continue to explore chains beyond this node.
     */
    public var visitsLeft = 0;

    /**
     * Return the name of this reaction, which is 'reaction_n',
     * where n is replaced by the reactionIndex. 
     * @return The name of this reaction.
     */
    override String getName() {
        return "reaction_" + this.reactionIndex;
    }
    
    /**
     * {@inheritDoc}
     */
    override ReactorInstance root() {
        parent.root()
    }

    /**
     * Return a descriptive string.
     */
    override toString() {
        getName + " of " + parent.getFullName
    }
}
