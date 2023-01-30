/** Representation of a runtime instance of a reaction. */

/*************
Copyright (c) 2019-2022, The University of California at Berkeley.

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

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;

import org.lflang.ASTUtils;
import org.lflang.TimeValue;
import org.lflang.lf.Action;
import org.lflang.lf.BuiltinTriggerRef;
import org.lflang.lf.Port;
import org.lflang.lf.Reaction;
import org.lflang.lf.Timer;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;

/**
 * Representation of a compile-time instance of a reaction.
 * Like {@link ReactorInstance}, if one or more parents of this reaction
 * is a bank of reactors, then there will be more than one runtime instance
 * corresponding to this compile-time instance.  The {@link #getRuntimeInstances()}
 * method returns a list of these runtime instances, each an instance of the
 * inner class {@link ReactionInstance.Runtime}.  Each runtime instance has a "level", which is
 * its depth an acyclic precedence graph representing the dependencies between
 * reactions at a tag.
 *  
 * @author Edward A. Lee
 * @author Marten Lohstroh
 */
public class ReactionInstance extends NamedInstance<Reaction> {

    /**
     * Create a new reaction instance from the specified definition
     * within the specified parent. This constructor should be called
     * only by the ReactorInstance class, but it is public to enable unit tests.
     * @param definition A reaction definition.
     * @param parent The parent reactor instance, which cannot be null.
     * @param isUnordered Indicator that this reaction is unordered w.r.t. other reactions.
     * @param index The index of the reaction within the reactor (0 for the
     * first reaction, 1 for the second, etc.).
     */
    public ReactionInstance(
            Reaction definition, 
            ReactorInstance parent, 
            boolean isUnordered, 
            int index
    ) {
        super(definition, parent);
        this.index = index;
        this.isUnordered = isUnordered;
        
        // If the reaction body starts with the magic string
        // UNORDERED_REACTION_MARKER, then mark it unordered,
        // overriding the argument.
        String body = ASTUtils.toText(definition.getCode());
        if (body.contains(UNORDERED_REACTION_MARKER)) {
            this.isUnordered = true;
        }
        
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
                            portInstance = containedReactor.lookupPortInstance((Port)variable);
                            if (portInstance != null) {
                                this.sources.add(portInstance);
                                portInstance.dependentReactions.add(this);
                                this.triggers.add(portInstance);
                            }
                        }
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
            } else if (trigger instanceof BuiltinTriggerRef) {
                this.triggers.add(parent.getOrCreateBuiltinTrigger((BuiltinTriggerRef) trigger));
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
                        portInstance = containedReactor.lookupPortInstance((Port)variable);
                        if (portInstance != null) {
                            this.sources.add(portInstance);
                            portInstance.dependentReactions.add(this);
                            this.triggers.add(portInstance);
                        }
                    }
                }
            }
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
                    throw new InvalidSourceException(
                            "Unexpected effect. Cannot find port " + variable.getName());
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
    public Set<TriggerInstance<? extends Variable>> effects = new LinkedHashSet<>();

    /**
     * The ports, actions, or timers that this reaction is triggered by or uses.
     */
    public Set<TriggerInstance<? extends Variable>> sources = new LinkedHashSet<>();
    // FIXME: Above sources is misnamed because in the grammar,
    // "sources" are only the inputs a reaction reads without being
    // triggered by them. The name "reads" used here would be a better
    // choice in the grammar.
    
    /**
     * Deadline for this reaction instance, if declared.
     */
    public DeadlineInstance declaredDeadline;

    /**
     * Sadly, we have no way to mark reaction "unordered" in the AST,
     * so instead, we use a magic comment at the start of the reaction body.
     * This is that magic comment.
     */
    public static String UNORDERED_REACTION_MARKER
            = "**** This reaction is unordered.";

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
    public Set<TriggerInstance<? extends Variable>> reads = new LinkedHashSet<>();

    /**
     * The trigger instances (input ports, timers, and actions
     * that trigger reactions) that trigger this reaction.
     */
    public Set<TriggerInstance<? extends Variable>> triggers = new LinkedHashSet<>();

    //////////////////////////////////////////////////////
    //// Public methods.

    /**
     * Clear caches used in reporting dependentReactions() and dependsOnReactions().
     * This method should be called if any changes are made to triggers, sources,
     * or effects.
     * @param includingRuntimes If false, leave the runtime instances intact.
     *  This is useful for federated execution where levels are computed using
     *  the top-level connections, but then those connections are discarded.
     */
    public void clearCaches(boolean includingRuntimes) {
        dependentReactionsCache = null;
        dependsOnReactionsCache = null;
        if (includingRuntimes) runtimeInstances = null;
    }
    
    /**
     * Return the set of immediate downstream reactions, which are reactions
     * that receive data produced by this reaction plus
     * at most one reaction in the same reactor whose definition
     * lexically follows this one (unless this reaction is unordered).
     */
    public Set<ReactionInstance> dependentReactions() {
        // Cache the result.
        if (dependentReactionsCache != null) return dependentReactionsCache;
        dependentReactionsCache = new LinkedHashSet<>();
        
        // First, add the next lexical reaction, if appropriate.
        if (!isUnordered && parent.reactions.size() > index + 1) {
            // Find the next reaction in the parent's reaction list.
            dependentReactionsCache.add(parent.reactions.get(index + 1));
        }
        
        // Next, add reactions that get data from this one via a port.
        for (TriggerInstance<? extends Variable> effect : effects) {
            if (effect instanceof PortInstance) {
                for (SendRange senderRange
                        : ((PortInstance)effect).eventualDestinations()) {
                    for (RuntimeRange<PortInstance> destinationRange
                            : senderRange.destinations) {
                        dependentReactionsCache.addAll(
                                destinationRange.instance.dependentReactions);
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
        dependsOnReactionsCache = new LinkedHashSet<>();
        
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
                for (RuntimeRange<PortInstance> senderRange
                        : ((PortInstance)source).eventualSources()) {
                    dependsOnReactionsCache.addAll(senderRange.instance.dependsOnReactions);
                }
                // Then, add reactions that send directly to this port.
                dependsOnReactionsCache.addAll(source.dependsOnReactions);
            }
        }
        return dependsOnReactionsCache;
    }

    /**
     * Return a set of levels that runtime instances of this reaction have.
     * A ReactionInstance may have more than one level if it lies within
     * a bank and its dependencies on other reactions pass through multiports.
     */
    public Set<Integer> getLevels() {
        Set<Integer> result = new LinkedHashSet<>();
        // Force calculation of levels if it has not been done.
        // FIXME: Comment out this as I think it is redundant.
        //  If it is NOT redundant then deadline propagation is not correct
        // parent.assignLevels();
        for (Runtime runtime : runtimeInstances) {
            result.add(runtime.level);
        }
        return result;
    }
    
    /**
     * Return a set of deadlines that runtime instances of this reaction have.
     * A ReactionInstance may have more than one deadline if it lies within.
     */
    public Set<TimeValue> getInferredDeadlines() {
        Set<TimeValue> result = new LinkedHashSet<>();
        for (Runtime runtime : runtimeInstances) {
            result.add(runtime.deadline);
        }
        return result;
    }
        

    /**
     * Return a list of levels that runtime instances of this reaction have.
     * The size of this list is the total number of runtime instances.
     * A ReactionInstance may have more than one level if it lies within
     * a bank and its dependencies on other reactions pass through multiports.
     */
    public List<Integer> getLevelsList() {
        List<Integer> result = new LinkedList<>();
        // Force calculation of levels if it has not been done.
        // FIXME: Comment out this as I think it is redundant.
        //  If it is NOT redundant then deadline propagation is not correct
        // parent.assignLevels();
        for (Runtime runtime : runtimeInstances) {
            result.add(runtime.level);
        }
        return result;
    }
    
    /**
     * Return a list of the deadlines that runtime instances of this reaction have.
     * The size of this list is the total number of runtime instances.
     * A ReactionInstance may have more than one deadline if it lies within
     */
    public List<TimeValue> getInferredDeadlinesList() {
        List<TimeValue> result = new LinkedList<>();
        for (Runtime runtime : runtimeInstances) {
            result.add(runtime.deadline);
        }
        return result;
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
     * Return an array of runtime instances of this reaction in a
     * **natural order**, defined as follows.  The position within the
     * returned list of the runtime instance is given by a mixed-radix
     * number where the low-order digit is the bank index within the
     * container reactor (or 0 if it is not a bank), the second low order
     * digit is the bank index of the container's container (or 0 if
     * it is not a bank), etc., until the container that is directly
     * contained by the top level (the top-level reactor need not be
     * included because its index is always 0).
     * 
     * The size of the returned array is the product of the widths of all of the
     * container ReactorInstance objects. If none of these is a bank,
     * then the size will be 1.
     *     
     * This method creates this array the first time it is called, but then
     * holds on to it.  The array is used by {@link ReactionInstanceGraph}
     * to determine and record levels and deadline for runtime instances
     * of reactors.
     */
    public List<Runtime> getRuntimeInstances() {
        if (runtimeInstances != null) return runtimeInstances;
        int size = parent.getTotalWidth();
        // If the width cannot be determined, assume there is only one instance.
        if (size < 0) size = 1;
        runtimeInstances = new ArrayList<>(size);
        for (int i = 0; i < size; i++) {
            Runtime r = new Runtime();
            r.id = i;
            if (declaredDeadline != null) {
                r.deadline = declaredDeadline.maxDelay;
            }
            runtimeInstances.add(r);
        }
        return runtimeInstances;
    }

    /**
     * Purge 'portInstance' from this reaction, removing it from the list
     * of triggers, sources, effects, and reads.  Note that this leaves
     * the runtime instances intact, including their level information.
     */
    public void removePortInstance(PortInstance portInstance) {
        this.triggers.remove(portInstance);
        this.sources.remove(portInstance);
        this.effects.remove(portInstance);
        this.reads.remove(portInstance);
        clearCaches(false);
        portInstance.clearCaches();
    }
    
    /**
     * Return a descriptive string.
     */
    @Override
    public String toString() {
        return getName() + " of " + parent.getFullName();
    }

    /**
     * Determine logical execution time for each reaction during compile
     * time based on immediate downstream logical delays (after delays and actions)
     * and label each reaction with the minimum of all such delays.
     */
    public TimeValue assignLogicalExecutionTime() {
        if (this.let != null) {
            return this.let;
        }

        if (this.parent.isGeneratedDelay()) {
            return this.let = TimeValue.ZERO;
        }

        TimeValue let = null;

        // Iterate over effect and find minimum delay.
        for (TriggerInstance<? extends Variable> effect : effects) {
            if (effect instanceof PortInstance) {
                var afters = this.parent.getParent().children.stream().filter(c -> {
                    if (c.isGeneratedDelay()) {
                        return c.inputs.get(0).getDependsOnPorts().get(0).instance
                                       .equals((PortInstance) effect);
                    }
                    return false;
                }).map(c -> c.actions.get(0).getMinDelay())
                  .min(TimeValue::compare);
                
                if (afters.isPresent()) {
                    if (let == null) {
                        let = afters.get();
                    } else {
                        let = TimeValue.min(afters.get(), let);
                    }
                }
            } else if (effect instanceof ActionInstance) {
                var action = ((ActionInstance) effect).getMinDelay();
                if (let == null) {
                    let = action;
                } else {
                    let = TimeValue.min(action, let);
                }
            }
        }

        if (let == null) {
            let = TimeValue.ZERO;
        }
        return this.let = let;
    }

    //////////////////////////////////////////////////////
    //// Private variables.

    /** Cache of the set of downstream reactions. */
    private Set<ReactionInstance> dependentReactionsCache;

    /** Cache of the set of upstream reactions. */
    private Set<ReactionInstance> dependsOnReactionsCache;
    
    /**
     * Array of runtime instances of this reaction.
     * This has length 1 unless the reaction is contained
     * by one or more banks.  Suppose that this reaction
     * has depth 3, with full name r0.r1.r2.r. The top-level
     * reactor is r0, which contains r1, which contains r2,
     * which contains this reaction r.  Suppose the widths
     * of the containing reactors are w0, w1, and w2, and
     * we are interested in the instance at bank indexes
     * b0, b1, and b2.  That instance is in this array at
     * location given by the **natural ordering**, which
     * is the mixed radix number b2%w2; b1%w1.
     */
    private List<Runtime> runtimeInstances;

    private TimeValue let = null;

    ///////////////////////////////////////////////////////////
    //// Inner classes

    /** Inner class representing a runtime instance of a reaction. */
    public class Runtime {
        public TimeValue deadline;
        // If this reaction instance depends on exactly one upstream
        // reaction (via a port), then the "dominating" field will
        // point to that upstream reaction.
        public Runtime dominating;
        /** ID ranging from 0 to parent.getTotalWidth() - 1. */
        public int id;
        public int level;
        
        public ReactionInstance getReaction() {
            return ReactionInstance.this;
        }
        @Override
        public String toString() {
            String result = ReactionInstance.this + "(level: " + level;
            if (deadline != null && deadline != TimeValue.MAX_VALUE) {
               result += ", deadline: " + deadline;
            }
            if (dominating != null) {
                result += ", dominating: " + dominating.getReaction();
            }
            result += ")";
            return result;
        }

        public Runtime() {
            this.dominating = null;
            this.id = 0;
            this.level = 0;
            if (ReactionInstance.this.declaredDeadline != null) {
                this.deadline = ReactionInstance.this.declaredDeadline.maxDelay;
            } else {
                this.deadline = TimeValue.MAX_VALUE;
            }
        }
    }
}
