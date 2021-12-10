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

package org.lflang.generator;

import java.util.Iterator;
import java.util.LinkedHashSet;
import java.util.Set;

import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.lf.Action;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Timer;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;

/**
 * Representation of a runtime instance of a reaction.
 * A ReactionInstance object stores all dependency information necessary
 * for constructing a directed acyclic precedece graph.
 *  
 * @author{Edward A. Lee <eal@berkeley.edu>}
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
public class ReactionInstance extends NamedInstance<Reaction> {

    /**
     * Create a new reaction instance from the specified definition
     * within the specified parent. This constructor should be called
     * only by the ReactionInstance class.
     * @param definition A reaction definition.
     * @param parent The parent reactor instance, which cannot be null.
     * @param isUnordered Indicator that this reaction is unordered w.r.t. other reactions.
     * @param index The index of the reaction within the reactor (0 for the
     * first reaction, 1 for the second, etc.).
     */
    protected ReactionInstance(
            Reaction definition, 
            ReactorInstance parent, 
            boolean isUnordered, 
            int index
    ) {
        super(definition, parent);
        this.index = index;
        this.isUnordered = isUnordered;
        
        // If the reaction has no port triggers or sources, then
        // we can immediately assign it a level.
        // We also record it in the root reactor instance
        // so that other reactions can be assigned levels as well.
        boolean dependsOnPorts = false;
        
        // Identify the dependencies for this reaction.
        // First handle the triggers.
        for (TriggerRef trigger : definition.getTriggers()) {
            if (trigger instanceof VarRef) {
                Variable variable = ((VarRef)trigger).getVariable();
                if (variable instanceof Port) {
                    PortInstance portInstance = parent.lookupPortInstance((Port)variable);
                    // If the trigger is the port of a contained bank, then the
                    // portInstance will be null and we have to instead search for
                    // each port instance in the bank.
                    if (portInstance != null) {
                        this.sources.add(portInstance);
                        portInstance.dependentReactions.add(this);
                        this.triggers.add(portInstance);
                    } else if (((VarRef)trigger).getContainer() != null) {
                        // Port belongs to a contained reactor or bank.
                        ReactorInstance containedReactor 
                                = parent.lookupReactorInstance(((VarRef)trigger).getContainer());
                        if (containedReactor != null) {
                            if (containedReactor.bankMembers != null) {
                                // Contained reactor is a bank. Connect to all bank members.
                                for (ReactorInstance bankMember : containedReactor.bankMembers) {
                                    portInstance = bankMember.lookupPortInstance((Port)variable);
                                    if (portInstance != null) {
                                        this.sources.add(portInstance);
                                        portInstance.dependentReactions.add(this);
                                        this.triggers.add(portInstance);
                                    }
                                }
                            } else {
                                // Contained reactor is not a bank.
                                portInstance = containedReactor.lookupPortInstance((Port)variable);
                                if (portInstance != null) {
                                    this.sources.add(portInstance);
                                    portInstance.dependentReactions.add(this);
                                    this.triggers.add(portInstance);
                                }
                            }
                        }
                    }
                    // Mark this reaction as depending on a port and therefore not
                    // eligible to be assigned level 0. However,
                    // if the port is not connected and doesn't depend on any reactions,
                    // then it does not interfere with this reaction being given level 0.
                    if (portInstance != null
                            && (portInstance.dependsOnPorts.size() > 0 
                            || portInstance.dependsOnReactions.size() > 0)) {
                        dependsOnPorts = true;
                    }
                } else if (variable instanceof Action) {
                    var actionInstance = parent.lookupActionInstance(
                        (Action)((VarRef)trigger).getVariable());
                    this.triggers.add(actionInstance);
                    actionInstance.dependentReactions.add(this);
                    this.sources.add(actionInstance);
                } else if (variable instanceof Timer) {
                    var timerInstance = parent.lookupTimerInstance(
                            (Timer)((VarRef)trigger).getVariable());
                    this.triggers.add(timerInstance);
                    timerInstance.dependentReactions.add(this);
                    this.sources.add(timerInstance);
                }
            } else if (trigger.isStartup()) {
                this.triggers.add(parent.getOrCreateStartup(trigger));
            } else if (trigger.isShutdown()) {
                this.triggers.add(parent.getOrCreateShutdown(trigger));
            }
        }
        // Next handle the ports that this reaction reads.
        for (VarRef source : definition.getSources()) {
            Variable variable = source.getVariable();
            if (variable instanceof Port) {
                var portInstance = parent.lookupPortInstance((Port)variable);
                
                // If the trigger is the port of a contained bank, then the
                // portInstance will be null and we have to instead search for
                // each port instance in the bank.
                if (portInstance != null) {
                    this.sources.add(portInstance);
                    this.reads.add(portInstance);
                    portInstance.dependentReactions.add(this);
                } else if (source.getContainer() != null) {
                    ReactorInstance containedReactor
                            = parent.lookupReactorInstance(source.getContainer());
                    if (containedReactor != null) {
                        if (containedReactor.bankMembers != null) {
                            // Contained reactor is a bank. Connect to all members.
                            for (ReactorInstance bankMember : containedReactor.bankMembers) {
                                portInstance = bankMember.lookupPortInstance((Port)variable);
                                if (portInstance != null) {
                                    this.sources.add(portInstance);
                                    portInstance.dependentReactions.add(this);
                                    this.reads.add(portInstance);
                                }
                            }
                        } else {
                            // The trigger is a port of a contained reactor that is not a bank.
                            portInstance = containedReactor.lookupPortInstance((Port)variable);
                            if (portInstance != null) {
                                this.sources.add(portInstance);
                                portInstance.dependentReactions.add(this);
                                this.triggers.add(portInstance);
                            }
                        }
                    }
                }
                // Mark this reaction as depending on a port and therefore not
                // eligible to be assigned level 0. However,
                // if the port is not connected and doesn't depend on any reactions,
                // then it does not interfere with this reaction being given level 0.
                if (portInstance != null
                        && (portInstance.dependsOnPorts.size() > 0 
                        || portInstance.dependsOnReactions.size() > 0)) {
                    dependsOnPorts = true;
                }
            }
        }
        
        // Initialize the root's readyReactions queue, which it uses
        // to compute levels.
        if (!dependsOnPorts) {
            if (isUnordered || index == 0) {
                level = 0L;
            }
            root().reactionsWithLevels.add(this);
        }

        // Finally, handle the effects.
        for (VarRef effect : definition.getEffects()) {
            Variable variable = effect.getVariable();
            if (variable instanceof Port) {
                var portInstance = parent.lookupPortInstance(effect);
                if (portInstance != null) {
                    this.effects.add(portInstance);
                    portInstance.dependsOnReactions.add(this);
                } else {
                    // The effect container must be a bank of reactors.
                    // Need to find the ports of all the instances within the bank.
                    ReactorInstance bank = parent.lookupReactorInstance(effect.getContainer());
                    if (bank == null || bank.bankIndex != -2) {
                        throw new InvalidSourceException(
                                "Unexpected effect. Cannot find port " + variable.getName());
                    }
                    for (ReactorInstance bankElement : bank.bankMembers) {
                        portInstance = bankElement.lookupPortInstance((Port)variable);
                        this.effects.add(portInstance);
                        portInstance.dependsOnReactions.add(this);
                    }
                }
            } else if (variable instanceof Action) {
                // Effect is an Action.
                var actionInstance = parent.lookupActionInstance(
                    (Action)variable);
                this.effects.add(actionInstance);
                actionInstance.dependsOnReactions.add(this);
            } else {
                // Effect is either a mode or an unresolved reference.
                // Do nothing, transitions will be set up by the ModeInstance.
            }
        }
        // Create a deadline instance if one has been defined.
        if (this.definition.getDeadline() != null) {
            this.declaredDeadline = new DeadlineInstance(
                this.definition.getDeadline(), this);
        }
    }

    //////////////////////////////////////////////////////
    //// Public fields.

    /** 
     * Indicates the chain this reaction is a part of. It is constructed
     * through a bit-wise or among all upstream chains. Each fork in the
     * dependency graph setting a new, unused bit to true in order to
     * disambiguate it from parallel chains. Note that zero results in
     * no overlap with any other reaction, which means the reaction can
     * execute in parallel with any other reaction. The default is 1L.
     * If left at the default, parallel execution will be based purely
     * on levels.
     */
    public long chainID = 1L;

    /**
     * The ports or actions that this reaction may write to.
     */
    public Set<TriggerInstance<? extends Variable>> effects 
            = new LinkedHashSet<TriggerInstance<? extends Variable>>();

    /**
     * The ports, actions, or timers that this reaction is triggered by or uses.
     */
    public Set<TriggerInstance<? extends Variable>> sources 
            = new LinkedHashSet<TriggerInstance<? extends Variable>>();
    // FIXME: Above sources is misnamed because in the grammar,
    // "sources" are only the inputs a reaction reads without being
    // triggered by them. The name "reads" used here would be a better
    // choice in the grammer.
    
    /**
     * Deadline for this reaction instance, if declared.
     */
    public DeadlineInstance declaredDeadline;

    /**
     * Inferred deadline. Defaults to the maximum long value.
     */
    public TimeValue deadline = new TimeValue(TimeValue.MAX_LONG_DEADLINE, TimeUnit.NANO);

    /**
     * The level in the dependence graph. -1 indicates that the level
     * has not yet been assigned.
     */
    public long level = -1L;

    /**
     * Index of order of occurrence within the reactor definition.
     * The first reaction has index 0, the second index 1, etc.
     */
    public int index;

    /**
     * Whether or not this reaction is ordered with respect to other
     * reactions in the same reactor.
     */
    public boolean isUnordered;

    /**
     * The ports that this reaction reads but that do not trigger it.
     */
    public Set<TriggerInstance<? extends Variable>> reads
            = new LinkedHashSet<TriggerInstance<? extends Variable>>();

    /**
     * The trigger instances (input ports, timers, and actions
     * that trigger reactions) that trigger this reaction.
     */
    public Set<TriggerInstance<? extends Variable>> triggers
            = new LinkedHashSet<TriggerInstance<? extends Variable>>();

    /**
     * Sources through which this reaction instance has been visited.
     */
    public Set<ReactionInstance> visited = new LinkedHashSet<ReactionInstance>();

    /**
     * Counter that indicates how many times this node has been visited during
     * the graph traversal that sets the chainIDs. Only when this counter hits zero
     * shall the traversal continue to explore chains beyond this node.
     */
    public int visitsLeft = 0;

    //////////////////////////////////////////////////////
    //// Public methods.

    /**
     * Clear caches used in reporting dependentReactions() and dependsOnReactions().
     * This method should be called if any changes are made to triggers, sources,
     * or effects.
     */
    public void clearCaches() {
        dependentReactionsCache = null;
        dependsOnReactionsCache = null;
    }
    
    /**
     * Return the set of immediate downstream reactions, which are reactions
     * that receive data produced produced by this reaction plus
     * at most one reaction in the same reactor whose definition
     * lexically follows this one (unless this reaction is unordered).
     */
    public Set<ReactionInstance> dependentReactions() {
        // Cache the result.
        if (dependentReactionsCache != null) return dependentReactionsCache;
        dependentReactionsCache = new LinkedHashSet<ReactionInstance>();
        
        // First, add the next lexical reaction, if appropriate.
        if (!isUnordered && parent.reactions.size() > index + 1) {
            // Find the next reaction in the parent's reaction list.
            dependentReactionsCache.add(parent.reactions.get(index + 1));
        }
        
        // Next, add reactions that get data from this one via a port.
        for (TriggerInstance<? extends Variable> effect : effects) {
            if (effect instanceof PortInstance) {
                for (PortInstance.SendRange senderRange
                        : ((PortInstance)effect).eventualDestinations()) {
                    for (PortInstance.Range destinationRange
                            : senderRange.destinations) {
                        dependentReactionsCache.addAll(
                                destinationRange.getPortInstance().dependentReactions);
                    }
                }
            }
        }
        return dependentReactionsCache;
    }
    
    /**
     * Return the set of immediate upstream reactions, which are reactions
     * that send data to this one plus at most one reaction in the same
     * reactor whose definition immediately precedes the definition of this one
     * (unless this reaction is unordered).
     */
    public Set<ReactionInstance> dependsOnReactions() {
        // Cache the result.
        if (dependsOnReactionsCache != null) return dependsOnReactionsCache;
        dependsOnReactionsCache = new LinkedHashSet<ReactionInstance>();
        
        // First, add the previous lexical reaction, if appropriate.
        if (!isUnordered && index > 0) {
            // Find the previous ordered reaction in the parent's reaction list.
            int earlierIndex = index - 1;
            ReactionInstance earlierOrderedReaction = parent.reactions.get(earlierIndex);
            while (earlierOrderedReaction.isUnordered && --earlierIndex >= 0) {
                earlierOrderedReaction = parent.reactions.get(earlierIndex);
            }
            if (earlierIndex >= 0) {
                dependsOnReactionsCache.add(parent.reactions.get(index - 1));
            }
        }
        
        // Next, add reactions that send data to this one.
        for (TriggerInstance<? extends Variable> source : sources) {
            if (source instanceof PortInstance) {
                // First, add reactions that send data through an intermediate port.
                for (PortInstance.Range senderRange
                        : ((PortInstance)source).eventualSources()) {
                    dependsOnReactionsCache.addAll(senderRange.getPortInstance().dependsOnReactions);
                }
                // Then, add reactions that send directly to this port.
                dependsOnReactionsCache.addAll(source.dependsOnReactions);
            }
        }
        return dependsOnReactionsCache;
    }

    /**
     * Return the single dominating reaction if this reaction has one, or
     * null otherwise.
     */
    public ReactionInstance findSingleDominatingReaction() {
        if (dependsOnReactions().size() == 1) {
            Iterator<ReactionInstance> upstream = dependsOnReactionsCache.iterator();
            return upstream.next();
        }
        return null;
    }

    /**
     * Return the name of this reaction, which is 'reaction_n',
     * where n is replaced by the reaction index. 
     * @return The name of this reaction.
     */
    @Override
    public String getName() {
        return "reaction_" + this.index;
    }
    
    /**
     * Purge 'portInstance' from this reaction, removing it from the list
     * of triggers, sources, effects, and reads.
     */
    public void removePortInstance(PortInstance portInstance) {
        this.triggers.remove(portInstance);
        this.sources.remove(portInstance);
        this.effects.remove(portInstance);
        this.reads.remove(portInstance);
        clearCaches();
        portInstance.clearCaches();
    }
    
    /**
     * {@inheritDoc}
     */
    @Override
    public ReactorInstance root() {
        return parent.root();
    }

    /**
     * Return a descriptive string.
     */
    @Override
    public String toString() {
        return getName() + " of " + parent.getFullName();
    }

    //////////////////////////////////////////////////////
    //// Private variables.

    /** Cache of the set of downstream reactions. */
    private Set<ReactionInstance> dependentReactionsCache;

    /** Cache of the set of upstream reactions. */
    private Set<ReactionInstance> dependsOnReactionsCache;
}
