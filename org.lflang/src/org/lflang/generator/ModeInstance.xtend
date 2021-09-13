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

import java.util.LinkedList
import org.lflang.lf.Mode
import org.lflang.lf.VarRef

/**
 * Representation of a runtime instance of a mode.
 *  
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
class ModeInstance extends NamedInstance<Mode> {

    /**
     * Create a new reaction instance from the specified definition
     * within the specified parent. This constructor should be called
     * only by the ReactorInstance class after all other contents
     * (reactions, etc.) are registered because this constructor call
     * will look them up.
     * @param definition A mode definition.
     * @param parent The parent reactor instance, which cannot be null.
     */
    protected new(Mode definition, ReactorInstance parent) {
        super(definition, parent)
        
        collectMembers()
    }
    
    ////////////////////////////////////////////////////
    // Member fields.
    
    /** The action instances belonging to this mode instance. */
    public val actions = new LinkedList<ActionInstance>
    
    /** The reactor instances belonging to this mode instance, in order of declaration. */
    public val instantiations = new LinkedList<ReactorInstance>

    /** List of reaction instances for this reactor instance. */
    public val reactions = new LinkedList<ReactionInstance>

    /** The timer instances belonging to this reactor instance. */
    public val timers = new LinkedList<TimerInstance>
    
    /** The outgoing transitions of this mode. */
    public val transitions = new LinkedList<Transition>

    ////////////////////////////////////////////////////
    // Public methods.
    
    /**
     * Return the name of this mode.
     * @return The name of this mode.
     */
    override String getName() {
        return this.definition.name
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
    
    /**
     * Returns true iff this mode is the initial mode of this reactor instance.
     */
    def isInitial() {
        return definition.initial
    }
    
    /**
     * Sets up all transitions that leave this mode.
     * Requires that all mode instances and other contents
     * (reactions, etc.) of the parent reactor are created.
     */
    def setupTranstions() {
        transitions.clear()
        for (reaction : reactions) {
            for (effect : reaction.definition.effects) {
                if (effect instanceof VarRef) {
                    val target = effect.variable
                    if (target instanceof Mode) {
                        transitions += new Transition(this, parent.lookupModeInstance(target), reaction, effect)
                    }
                }
            }
        }
    }
    
    /**
     * Returns true iff this mode contains the given instance.
     */
    def boolean contains(NamedInstance<?> instance) {
        return switch(instance) {
            TimerInstance: timers.contains(instance)
            ActionInstance: actions.contains(instance)
            ReactorInstance: instantiations.contains(instance)
            ReactionInstance: reactions.contains(instance)
            default: false
        }
    }

    ////////////////////////////////////////////////////
    // Private methods.
    
    private def collectMembers() {
        // Collect timers
        for (decl : definition.timers) {
            val instance = parent.lookupTimerInstance(decl)
            if (instance !== null) {
                this.timers.add(instance)
            }
        }
        
        // Collect actions
        for (decl : definition.actions) {
            val instance = parent.lookupActionInstance(decl)
            if (instance !== null) {
                this.actions.add(instance)
            }
        }
        
        // Collect reactor instantiation
        for (decl : definition.instantiations) {
            val instance = parent.lookupReactorInstance(decl)
            if (instance !== null) {
                this.instantiations.add(instance)
            }
        }
        
        // Collect reactions
        for (decl : definition.reactions) {
            val instance = parent.lookupReactionInstance(decl)
            if (instance !== null) {
                this.reactions.add(instance)
            }
        }
    }

    ////////////////////////////////////////////////////
    // Data class.
        
    public static class Transition extends NamedInstance<VarRef> {
        public val ModeInstance source
        public val ModeInstance target
        public val ReactionInstance reaction
        
        new(ModeInstance source, ModeInstance target, ReactionInstance reaction, VarRef definition) {
            super(definition, source.parent)
            this.source = source
            this.target = target
            this.reaction = reaction
        }
        
        override getName() {
            return this.source.name + " -> " + this.target + " by " + this.reaction.name
        }
        
        override root() {
            return this.parent.root()
        }
        
        def getType() {
            return definition.modeTransitionType
        }
        
    }
    
}
