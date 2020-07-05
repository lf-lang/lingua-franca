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

package org.icyphy.generator

import java.util.HashSet
import org.icyphy.TimeValue
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.TriggerRef
import org.icyphy.linguaFranca.VarRef
import org.icyphy.linguaFranca.Variable

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
    protected new(Reaction definition, ReactorInstance parent, int index) {
        super(definition, parent);
        this.reactionIndex = index
        
        // Identify the dependencies for this reaction.
        // First handle the triggers.
        for (TriggerRef trigger : definition.triggers) {
            if (trigger instanceof VarRef) {
                if (trigger.variable instanceof Port) {
                    var portInstance = parent.getPortInstance(trigger)
                    this.dependsOnPorts.add(portInstance)
                    portInstance.dependentReactions.add(this)
                    this.triggers.add(portInstance)
                } else if (trigger.variable instanceof Action) {
                    var actionInstance = parent.getActionInstance(
                        trigger.variable as Action)
                    this.triggers.add(actionInstance)
                    actionInstance.dependentReactions.add(this)
                    this.dependsOnActions.add(actionInstance)
                } else if (trigger.variable instanceof Timer) {
                    var timerInstance = parent.getTimerInstance(
                        trigger.variable as Timer)
                    this.triggers.add(timerInstance)
                    timerInstance.dependentReactions.add(this)
                }
            } else if (trigger.isShutdown) {
                var actionInstance = parent.shutdownAction
                this.triggers.add(actionInstance)
                actionInstance.dependentReactions.add(this)
                this.dependsOnActions.add(actionInstance)
            } else if (trigger.isStartup) {
                var timerInstance = parent.startupTimer
                this.triggers.add(timerInstance)
                timerInstance.dependentReactions.add(this)
                this.dependsOnTimers.add(timerInstance)
            }
        }
        // Next handle the ports that this reaction reads.
        for (source : definition.sources) {
            if (source.variable instanceof Port) {
                var portInstance = parent.getPortInstance(source)
                this.dependsOnPorts.add(portInstance)
                this.reads.add(portInstance)
                portInstance.dependentReactions.add(this)
            }
        }

        // Finally, handle the effects.
        for (effect : definition.effects) {
            if (effect.variable instanceof Port) {
                var portInstance = parent.getPortInstance(effect)
                if (portInstance instanceof MultiportInstance) {
                    for (multiportInstance : portInstance.instances) {
                        this.dependentPorts.add(multiportInstance)
                        multiportInstance.dependsOnReactions.add(this)
                    }
                } else {
                    this.dependentPorts.add(portInstance)
                    portInstance.dependsOnReactions.add(this)
                }
            } else {
                // Effect must be an Action.
                var actionInstance = parent.getActionInstance(
                    effect.variable as Action)
                this.dependentActions.add(actionInstance)
                actionInstance.dependsOnReactions.add(this)
            }
        }
        // Create a deadline instance if one has been defined.
        if (this.definition.deadline !== null) {
            this.declaredDeadline = new DeadlineInstance(
                this.definition.deadline, this)
        }
    }

    /** 
     * Indicates the chain this reaction is a part of. It is constructed
     * through a bit-wise or among all upstream chains. Each fork in the
     * dependency graph setting a new, unused bit to true in order to
     * disambiguate it from parallel chains.
     */
    public long chainID = 0L;

    /** The actions that this reaction triggers. */
    public var dependentActions = new HashSet<ActionInstance>();

    /** The ports that this reaction may write to. */
    public var dependentPorts = new HashSet<PortInstance>();

    /** The reactions that depend on this reaction. */
    public var dependentReactions = new HashSet<ReactionInstance>();

    /** The actions that this reaction is triggered by. */
    public var dependsOnActions = new HashSet<ActionInstance>();

    /** The ports that this reaction is triggered by or uses. */
    public var dependsOnPorts = new HashSet<PortInstance>();

    /** The timers that this reaction is triggered by. */
    public var dependsOnTimers = new HashSet<TimerInstance>();

    /** The reactions that this reaction depends on. */
    public var dependsOnReactions = new HashSet<ReactionInstance>();

    /** Deadline for this reaction instance, if declared.*/
    public DeadlineInstance declaredDeadline

    /** Inferred deadline. Defaults to the maximum long value. */
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
     * The ports that this reaction reads but that do not trigger it.
     */
    public var reads = new HashSet<TriggerInstance<Variable>>

    /**
     * The trigger instances (input ports, timers, and actions
     * that trigger reactions) that trigger this reaction.
     */
    public var triggers = new HashSet<TriggerInstance<Variable>>

    /**
     * Sources through which this reaction instance has been visited.
     */
    public var visited = new HashSet<ReactionInstance>

    /**
     * Return the name of this reaction, which is 'reaction_n',
     * where n is replaced by the reactionIndex. 
     * @return The name of this reaction.
     */
    override String getName() {
        return "reaction_" + this.reactionIndex;
    }

    /**
     * Return the main reactor, which is the top-level parent.
     * @return The top-level parent.
     */
    override ReactorInstance main() {
        parent.main
    }

    /** Return a descriptive string. */
    override toString() {
        getName + " of " + parent.getFullName
    }
}
