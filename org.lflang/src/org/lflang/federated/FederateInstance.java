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

package org.lflang.federated;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.TimeValue;
import org.lflang.generator.ActionInstance;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.TriggerInstance;
import org.lflang.lf.Action;
import org.lflang.lf.ActionOrigin;
import org.lflang.lf.Expression;
import org.lflang.lf.Input;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Output;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.Timer;
import org.lflang.lf.TriggerRef;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;

import com.google.common.base.Objects;


/** 
 * Instance of a federate, or marker that no federation has been defined
 * (if isSingleton() returns true). Every top-level reactor (contained
 * directly by the main reactor) is a federate, so there will be one
 * instance of this class for each top-level reactor.
 * 
 * @author{Edward A. Lee <eal@berkeley.edu>}
 */
public class FederateInstance {

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
    public FederateInstance(
            Instantiation instantiation, 
            int id, 
            int bankIndex, 
            GeneratorBase generator, 
            ErrorReporter errorReporter) {
        this.instantiation = instantiation;
        this.id = id;
        this.generator = generator;
        this.bankIndex = bankIndex;
        this.errorReporter = errorReporter;
                
        if (instantiation != null) {
            this.name = instantiation.getName();
            // If the instantiation is in a bank, then we have to append
            // the bank index to the name.
            if (instantiation.getWidthSpec() != null) {
                this.name = instantiation.getName() + "__" + bankIndex;
            }
        }
    }

    /////////////////////////////////////////////
    //// Public Fields
    
    /**
     * The position within a bank of reactors for this federate.
     * This is 0 if the instantiation is not a bank of reactors.
     */
    public int bankIndex = 0;
    
    /**
     * A list of outputs that can be triggered directly or indirectly by physical actions.
     */
    public Set<Expression> outputsConnectedToPhysicalActions = new LinkedHashSet<>();
    
    /**
     * The host, if specified using the 'at' keyword.
     */
    public String host = "localhost";
    
    /**
     * The instantiation of the top-level reactor, or null if there is no federation.
     */
    public Instantiation instantiation;
    
    /**
     * Map from the federates that this federate receives messages from
     * to the delays on connections from that federate. The delay set
     * may may include null, meaning that there is a connection
     * from the federate instance that has no delay.
     */
    public Map<FederateInstance, Set<Expression>> dependsOn = new LinkedHashMap<>();
    
    /**
     * The directory, if specified using the 'at' keyword.
     */
    public String dir = null;
    
    /**
     * The port, if specified using the 'at' keyword.
     */
    public int port = 0;
    
    /**
     * Map from the federates that this federate sends messages to
     * to the delays on connections to that federate. The delay set
     * may may include null, meaning that there is a connection
     * from the federate instance that has no delay.
     */
    public Map<FederateInstance, Set<Expression>> sendsTo = new LinkedHashMap<>();
    
    /**
     * The user, if specified using the 'at' keyword.
     */
    public String user = null;
    
    /**
     * The integer ID of this federate.
     */
    public int id = 0;
    
    /**
     * The name of this federate instance. This will be the instantiation
     * name, poassibly appended with "__n", where n is the bank position of
     * this instance if the instantiation is of a bank of reactors.
     */
    public String name = "Unnamed";
    
    /**
     * List of networkMessage actions. Each of these handles a message
     *  received from another federate over the network. The ID of
     *  receiving port is simply the position of the action in the list.
     *  The sending federate needs to specify this ID.
     */
    public List<Action> networkMessageActions = new ArrayList<>();
    
    /**
     * A set of federates with which this federate has an inbound connection
     * There will only be one physical connection even if federate A has defined multiple
     * physical connections to federate B. The message handler on federate A will be
     * responsible for including the appropriate information in the message header (such as port ID)
     * to help the receiver distinguish different events.
     */
    public Set<FederateInstance> inboundP2PConnections = new LinkedHashSet<>();
    
    /**
     * A list of federate with which this federate has an outbound physical connection.
     * There will only be one physical connection even if federate A has defined multiple
     * physical connections to federate B. The message handler on federate B will be
     * responsible for distinguishing the incoming messages by parsing their header and
     * scheduling the appropriate action.
     */
    public Set<FederateInstance> outboundP2PConnections = new LinkedHashSet<>();
    
    /**
     * A list of triggers for network input control reactions. This is used to trigger
     * all the input network control reactions that might be nested in a hierarchy.
     */
    public List<Action> networkInputControlReactionsTriggers = new ArrayList<>();
    
    /**
     * The trigger that triggers the output control reaction of this
     * federate.
     * 
     * The network output control reactions send a PORT_ABSENT message for a network output port,
     * if it is absent at the current tag, to notify all downstream federates that no value will
     * be present on the given network port, allowing input control reactions on those federates
     * to stop blocking.
     */
    public Variable networkOutputControlReactionsTrigger = null;
    
    /**
     * Indicates whether the federate is remote or local
     */
    public boolean isRemote = false;
    
    /**
     * List of generated network reactions (network receivers,
     * network input control reactions, network senders, and network output control
     * reactions) that belong to this federate instance.
     */
    public List<Reaction> networkReactions = new ArrayList<>();
    
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
    public List<VarRef> remoteNetworkReactionTriggers = new ArrayList<>();

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
    public boolean contains(Action action) {
        Reactor reactor  = ASTUtils.getEnclosingReactor(action);
        if (!reactor.isFederated() || isSingleton()) return true;
        
        // If the action is used as a trigger, a source, or an effect for a top-level reaction
        // that belongs to this federate, then generate it.
        for (Reaction react : ASTUtils.allReactions(reactor)) {
            if (contains(react)) {
                // Look in triggers
                for (TriggerRef trigger : convertToEmptyListIfNull(react.getTriggers())) {
                    if (trigger instanceof VarRef) {
                        VarRef triggerAsVarRef = (VarRef) trigger;
                        if (Objects.equal(triggerAsVarRef.getVariable(), (Variable) action)) {
                            return true;
                        }
                    }
                }
                // Look in sources
                for (VarRef source : convertToEmptyListIfNull(react.getSources())) {
                    if (Objects.equal(source.getVariable(), (Variable) action)) {
                        return true;
                    }
                }
                // Look in effects
                for (VarRef effect : convertToEmptyListIfNull(react.getEffects())) {
                    if (Objects.equal(effect.getVariable(), (Variable) action)) {
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
     * NOTE: This method assumes that it will not be called with reaction arguments
     * that are within other federates. It should only be called on reactions that are
     * either at the top level or within this federate. For this reason, for any reaction
     * not at the top level, it returns true.
     *
     * @param reaction The reaction.
     */
    public boolean contains(Reaction reaction) {
        Reactor reactor  = ASTUtils.getEnclosingReactor(reaction);
        if (!reactor.isFederated() || this.isSingleton()) return true;
        
        if (!reactor.getReactions().contains(reaction)) return false;
        
        if (networkReactions.contains(reaction)) {
            // Reaction is a network reaction that belongs to this federate
            return true;
        }
        
        int reactionBankIndex = generator.getReactionBankIndex(reaction);
        if (reactionBankIndex >= 0 && this.bankIndex >= 0 && reactionBankIndex != this.bankIndex) {
            return false;
        }
        
        // If this has been called before, then the result of the
        // following check is cached.
        if (excludeReactions != null) {
            return !excludeReactions.contains(reaction);
        }
        
        indexExcludedTopLevelReactions(reactor);
       
        return !excludeReactions.contains(reaction);
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
    public boolean contains(ReactorInstance instance) {
        if (isSingleton()) {
            return instance != null;
        }
        if (instance.getParent() == null) {
            return true; // Top-level reactor
        }
        // Start with this instance, then check its parents.
        ReactorInstance i = instance;
        while (i != null) {
            if (i.getDefinition() == instantiation) {
                return true;
            }
            i = i.getParent();
        }
        return false;
    }
    
    /**
     * Return true if the specified timer should be included in the code generated
     * for the federate. This means that the timer is used as a trigger
     * in a top-level reaction that belongs to this federate.
     * This also returns true if the program is not federated.
     * 
     * @param timer The timer
     * @return True if this federate contains the action in the specified reactor
     */
    public boolean contains(Timer timer) {
        Reactor reactor  = ASTUtils.getEnclosingReactor(timer);
        if (!reactor.isFederated() || this.isSingleton()) return true;
        
        // If the action is used as a trigger, a source, or an effect for a top-level reaction
        // that belongs to this federate, then generate it.
        for (Reaction r : ASTUtils.allReactions(reactor)) {
            if (contains(r)) {
                // Look in triggers
                for (TriggerRef trigger : convertToEmptyListIfNull(r.getTriggers())) {
                    if (trigger instanceof VarRef) {
                        VarRef triggerAsVarRef = (VarRef) trigger;
                        if (Objects.equal(triggerAsVarRef.getVariable(), (Variable) timer)) {
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
    public int numRuntimeInstances(ReactorInstance reactor) {
        if (!contains(reactor)) return 0;
        int depth = this.isSingleton() ? 0 : 1;
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
    private void indexExcludedTopLevelReactions(Reactor federatedReactor) {
        boolean inFederate = false;
        if (excludeReactions != null) {
            throw new IllegalStateException("The index for excluded reactions at the top level is already built.");
        }

        excludeReactions = new LinkedHashSet<Reaction>();

        // Construct the set of excluded reactions for this federate.
        // If a reaction is a network reaction that belongs to this federate, we
        // don't need to perform this analysis.
        Iterable<Reaction> reactions = IterableExtensions.filter(ASTUtils.allReactions(federatedReactor), it -> { return !networkReactions.contains(it); });
        for (Reaction react : reactions) {
            // Create a collection of all the VarRefs (i.e., triggers, sources, and effects) in the react 
            // signature that are ports that reference federates.
            // We then later check that all these VarRefs reference this federate. If not, we will add this
            // react to the list of reactions that have to be excluded (note that mixing VarRefs from
            // different federates is not allowed).
            List<VarRef> allVarRefsReferencingFederates = new ArrayList<VarRef>();
            // Add all the triggers that are outputs
            Stream<VarRef> triggersAsVarRef = react.getTriggers().stream().filter(it -> it instanceof VarRef).map(it -> (VarRef) it);
            allVarRefsReferencingFederates.addAll(
                triggersAsVarRef.filter(it -> it.getVariable() instanceof Output).collect(Collectors.toList())
            );
            // Add all the sources that are outputs
            allVarRefsReferencingFederates.addAll(
                react.getSources().stream().filter(it -> it.getVariable() instanceof Output).collect(Collectors.toList())
            );
            // Add all the effects that are inputs
            allVarRefsReferencingFederates.addAll(
                react.getEffects().stream().filter(it -> it.getVariable() instanceof Input).collect(Collectors.toList())
            );
            inFederate = containsAllVarRefs(allVarRefsReferencingFederates);
            if (!inFederate) {
                excludeReactions.add(react);
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
    private boolean containsAllVarRefs(Iterable<VarRef> varRefs) {
        var referencesFederate = false;
        var inFederate = true;
        for (VarRef varRef : varRefs) {
            if (varRef.getContainer() == this.instantiation) {
                referencesFederate = true;
            } else {
                if (referencesFederate) {
                    errorReporter.reportError(varRef, "Mixed triggers and effects from" +
                        " different federates. This is not permitted");
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
    public boolean isSingleton() {
        return ((instantiation == null) || (generator.federates.size() <= 1));
    }
     
    /**
     * Find output ports that are connected to a physical action trigger upstream
     * in the same reactor. Return a list of such outputs paired with the minimum delay
     * from the nearest physical action.
     * @param instance The reactor instance containing the output ports
     * @return A LinkedHashMap<Output, TimeValue>
     */
    public LinkedHashMap<Output, TimeValue> findOutputsConnectedToPhysicalActions(ReactorInstance instance) {
        LinkedHashMap<Output, TimeValue> physicalActionToOutputMinDelay = new LinkedHashMap<>();
        // Find reactions that write to the output port of the reactor
        for (PortInstance output : instance.outputs) {
            for (ReactionInstance reaction : output.getDependsOnReactions()) {
                TimeValue minDelay = findNearestPhysicalActionTrigger(reaction);
                if (!Objects.equal(minDelay, TimeValue.MAX_VALUE)) {
                    physicalActionToOutputMinDelay.put((Output) output.getDefinition(), minDelay);
                }
            }
        }
        return physicalActionToOutputMinDelay;
    }
    
    @Override
    public String toString() {
        return "Federate " + id + ": "
            + ((instantiation != null) ? instantiation.getName() : "no name");
    }

    /////////////////////////////////////////////
    //// Private Fields
     
    /**
     * Cached result of analysis of which reactions to exclude from main.
     */
    private Set<Reaction> excludeReactions = null;
    
    /**
     * The generator using this.
     */
    private GeneratorBase generator = null;
    
    /**
     * Returns the generator that is using this federate instance
     */
    public GeneratorBase getGenerator() {
        return this.generator;
    }
    
    /**
     * An error reporter
     */
    private final ErrorReporter errorReporter;
    
    /**
     * Find the nearest (shortest) path to a physical action trigger from this
     * 'reaction' in terms of minimum delay.
     * 
     * @param reaction The reaction to start with
     * @return The minimum delay found to the nearest physical action and
     *  TimeValue.MAX_VALUE otherwise
     */
    public TimeValue findNearestPhysicalActionTrigger(ReactionInstance reaction) {
        TimeValue minDelay = TimeValue.MAX_VALUE;
        for (TriggerInstance<? extends Variable> trigger : reaction.triggers) {
            if (trigger.getDefinition() instanceof Action) {
                Action action = (Action) trigger.getDefinition();
                ActionInstance actionInstance = (ActionInstance) trigger;
                if (action.getOrigin() == ActionOrigin.PHYSICAL) {
                    if (actionInstance.getMinDelay().isEarlierThan(minDelay)) {
                        minDelay = actionInstance.getMinDelay();
                    }
                } else if (action.getOrigin() == ActionOrigin.LOGICAL) {
                    // Logical action
                    // Follow it upstream inside the reactor
                    for (ReactionInstance uReaction: actionInstance.getDependsOnReactions()) {
                        // Avoid a loop
                        if (!Objects.equal(uReaction, reaction)) {
                            TimeValue uMinDelay = actionInstance.getMinDelay().add(findNearestPhysicalActionTrigger(uReaction));
                            if (uMinDelay.isEarlierThan(minDelay)) {
                                minDelay = uMinDelay;
                            }
                        }
                    }
                }
                
            } else if (trigger.getDefinition() instanceof Output) {
                // Outputs of contained reactions
                PortInstance outputInstance = (PortInstance) trigger;
                for (ReactionInstance uReaction: outputInstance.getDependsOnReactions()) {
                    TimeValue uMinDelay = findNearestPhysicalActionTrigger(uReaction);
                    if (uMinDelay.isEarlierThan(minDelay)) {
                        minDelay = uMinDelay;
                    }
                }
            }
        }
        return minDelay;
    }
    
    /**
     * Remove triggers in this federate's network reactions that are defined in remote federates.
     */
    public void removeRemoteFederateConnectionPorts() {
        for (Reaction reaction : this.networkReactions) {
          reaction.getTriggers().removeAll(this.remoteNetworkReactionTriggers);
        }
    }
    
    // TODO: Put this function into a utils file instead
    private <T extends Object> List<T> convertToEmptyListIfNull(List<T> list) {
        return list == null ? new ArrayList<>() : list;
    }
}
