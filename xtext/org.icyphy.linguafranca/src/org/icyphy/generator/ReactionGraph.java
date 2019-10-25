package org.icyphy.generator;

import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.Set;

import org.eclipse.emf.common.util.EList;
import org.icyphy.linguaFranca.Input;
import org.icyphy.linguaFranca.Output;
import org.icyphy.linguaFranca.Reaction;
import org.icyphy.linguaFranca.VarRef;

/**
 * Precedence graph analysis for Lingua Franca models. The way to use this class
 * is to call calculateLevels() after creating the graph. Upon completion, you
 * can retrieve the levels by calling getReactionInstance() to get the
 * ReactionInstance, which has a level field.
 */
public class ReactionGraph {

    /**
     * Create a new precedence graph for reactions for the specified code
     * generator.
     * 
     * @param generator The code generator.
     */
    public ReactionGraph(GeneratorBase generator) {
        _generator = generator;
    }

    /** All nodes in the graph. */
    public LinkedHashSet<ReactionInstance> nodes = new LinkedHashSet<ReactionInstance>();

    /**
     * Add the specified reactor instance and all its contained reactor
     * instances to the graph. This will create a reaction instance for each
     * reaction and establish the dependencies between reactions and ports and
     * between ports. That reaction instance can be retrieved using
     * {@link getReactionInstance(Instance, Reaction)} or by iterating over
     * nodes.
     * 
     * @param reactorInstance The reactor instance.
     */
    private void addReactorInstance(ReactorInstance reactorInstance) {
        // In the first pass, create a ReactionInstance for each reaction and
        // a PortInstance for each port on which reactions depend and for each
        // port that a reaction writes to.

        // Store the reaction instances in a list index by the defining reactor
        // instance.
        LinkedHashMap<Reaction, ReactionInstance> reactionInstances = _reactorToReactionList
                .get(reactorInstance);
        if (reactionInstances == null) {
            reactionInstances = new LinkedHashMap<Reaction, ReactionInstance>();
            _reactorToReactionList.put(reactorInstance, reactionInstances);
        }

        EList<Reaction> reactions = reactorInstance.definition.getReactorClass()
                .getReactions();
        if (reactions != null) {
            ReactionInstance previousReaction = null;
            for (Reaction reaction : reactions) {
                // Create the reaction instance.
                ReactionInstance reactionInstance = new ReactionInstance(
                        reaction, reactorInstance);
                // If there is an earlier reaction in this same reactor, then
                // create a link
                // in the dependence graph.
                if (previousReaction != null) {
                    previousReaction.dependentReactions.add(reactionInstance);
                    reactionInstance.dependsOnReactions.add(previousReaction);
                }
                previousReaction = reactionInstance;
                // Add the reaction instance to the map of reactions for this
                // reactor.
                reactionInstances.put(reactionInstance.reactionSpec,
                        reactionInstance);
                nodes.add(reactionInstance);

                // If the reaction is triggered by an input to this reactor
                // instance,
                // then create a PortInstance for that port (if it does not
                // already exist)
                // and establish the dependency on that port.
                // Only consider inputs and outputs, ignore actions and timers.
                EList<VarRef> deps = null;
                // First handle dependencies
                if (reaction.getTriggers() != null) {
                    deps = reaction.getTriggers();
                }
                if (reaction.getSources() != null) {
                    if (deps != null) {
                        deps.addAll(reaction.getSources());
                    } else {
                        deps = reaction.getSources();
                    }
                }
                if (deps != null) {
                    for (VarRef ref : deps) {
                        if (ref.getVariable() instanceof Input) {
                            Input input = (Input) ref.getVariable();
                            PortInstance port = new PortInstance(
                                    reactorInstance, input);
                            reactorInstance.portInstances.put(input, port);
                            port.dependentReactions.add(reactionInstance);
                            reactionInstance.dependsOnPorts.add(port);
                        } else if (ref.getVariable() instanceof Output) {
                            ReactorInstance childInstance = reactorInstance
                                    .getChildReactorInstance(
                                            ref.getContainer());
                            if (childInstance == null) {
                                // Scope provider should catch this
                                _generator.reportError(ref,
                                        "Invalid trigger `"
                                                + ref.getVariable().getName()
                                                + "` for reaction in reactor `"
                                                + reactorInstance.definition
                                                        .getReactorClass()
                                                        .getName());
                            } else {
                                Output output = (Output) ref.getVariable();
                                PortInstance port = childInstance.portInstances
                                        .get(output);
                                if (port == null) {
                                    port = new PortInstance(childInstance,
                                            output);
                                    childInstance.portInstances.put(output,
                                            port);
                                }
                                port.dependentReactions.add(reactionInstance);
                                reactionInstance.dependentPorts.add(port);
                            }

                        }
                    }
                }

                // Then handle anti-dependencies
                // If the reaction produces an output from this reactor
                // instance,
                // then create a PortInstance for that port (if it does not
                // already exist)
                // and establish the dependency on that port.
                if (reaction.getEffects() != null) {
                    for (VarRef antidep : reaction.getEffects()) {
                        // Check for dotted output, which is the input of a
                        // contained reactor.
                        if (antidep.getVariable() instanceof Input) {
                            ReactorInstance childInstance = reactorInstance
                                    .getChildReactorInstance(
                                            antidep.getContainer());
                            if (childInstance == null) {
                                // FIXME: Remove? Scope provider should catch
                                // this
                                _generator.reportError(antidep,
                                        "Invalid trigger `"
                                                + antidep.getVariable()
                                                        .getName()
                                                + "` for reaction in reactor `"
                                                + reactorInstance.definition
                                                        .getReactorClass()
                                                        .getName());
                            } else {
                                Input input = (Input) antidep.getVariable();
                                PortInstance port = childInstance.portInstances
                                        .get(input);
                                if (port == null) {
                                    port = new PortInstance(childInstance,
                                            input);
                                    childInstance.portInstances.put(input,
                                            port);
                                }
                                port.dependsOnReactions.add(reactionInstance);
                                reactionInstance.dependentPorts.add(port);
                            }
                        } else if (antidep.getVariable() instanceof Output) {
                            Output output = (Output) antidep.getVariable();
                            PortInstance port = reactorInstance.portInstances
                                    .get(output);
                            if (port == null) {
                                port = new PortInstance(reactorInstance,
                                        output);
                                reactorInstance.portInstances.put(output, port);
                            }
                            port.dependsOnReactions.add(reactionInstance);
                            reactionInstance.dependentPorts.add(port);
                        }
                    }
                }
            }
        }
        // Next, iterate over all contained reactors to repeat the above.
        for (ReactorInstance containedReactor : reactorInstance.children) {
            addReactorInstance(containedReactor);
        }

        // Next, iterate over all connections to establish dependencies between
        // ports.
        for (PortInstance source : reactorInstance.destinations.keySet()) {
            for (PortInstance destination : reactorInstance.destinations
                    .get(source)) {
                destination.dependsOnPorts.add(source);
                source.dependentPorts.add(destination);
            }
        }
    }

    /**
     * Calculate the levels for the graph.
     * 
     * @param main The top-level reactor instance for the model.
     */
    public void calculateLevels(ReactorInstance main) {
        // Create a dependence graph (this includes ports).
        addReactorInstance(main);
        // Collapse the dependence graph to only include reactions.
        collapseDependencies();
        // Calculate levels.
        // Copy the node list so we can remove elements from it.
        LinkedHashSet<ReactionInstance> nodesCopy = new LinkedHashSet<ReactionInstance>();
        nodesCopy.addAll(nodes);
        int level = 0;
        while (!nodesCopy.isEmpty()) {
            if (_independentReactions.isEmpty()) {
                StringBuilder cycleReactors = new StringBuilder();
                for (ReactionInstance instance : nodesCopy) {
                    if (cycleReactors.length() > 0) {
                        cycleReactors.append(", ");
                    }
                    cycleReactors
                            .append(instance.reactorInstance.getFullName());
                }
                _generator.reportError(main.definition.getReactorClass(),
                        "Dependency graph has cycles including: "
                                + cycleReactors.toString());
                return;
            }
            // Copy the _independentReactions list so we can remove nodes from
            // it
            // and add more to it.
            Set<ReactionInstance> copy = new HashSet<ReactionInstance>();
            copy.addAll(_independentReactions);
            for (ReactionInstance independentReaction : copy) {
                _independentReactions.remove(independentReaction);
                nodesCopy.remove(independentReaction);
                independentReaction.level = level;

                // Remove the backward dependence of subsequent reactions.
                for (ReactionInstance dependentReaction : independentReaction.dependentReactions) {
                    dependentReaction.dependsOnReactions
                            .remove(independentReaction);
                    // If this is the last reaction it depends on, add to the
                    // list for the next round.
                    if (dependentReaction.dependsOnReactions.isEmpty()) {
                        _independentReactions.add(dependentReaction);
                    }
                }
            }
            level += 1;
        }
    }

    /**
     * Collapse all the port dependencies into dependencies between reactions
     * only. As a side effect, this collects the independent reactions to be
     * used by calculateLevels().
     */
    public void collapseDependencies() {
        for (ReactionInstance reaction : nodes) {
            reaction.collapseDependencies();
        }
    }

    /**
     * Get the reaction instance.
     * 
     * @param reactorInstance The reactor instance for the reaction.
     * @param reaction        The reaction specification in the AST.
     * @return The reaction instance or null if there is none.
     * @exception If the reaction instance is not in the graph.
     */
    public ReactionInstance getReactionInstance(ReactorInstance reactorInstance,
            Reaction reaction) throws Exception {
        LinkedHashMap<Reaction, ReactionInstance> reactionInstances = _reactorToReactionList
                .get(reactorInstance);
        if (reactionInstances == null) {
            throw new Exception("Reaction instance is not in the graph.");
        }
        return reactionInstances.get(reaction);
    }

    //////////////////////////////////////////////////////////////////////////
    //// Private fields

    /** The code generator using this graph. */
    private GeneratorBase _generator;

    /** Set of independent reactions. */
    private Set<ReactionInstance> _independentReactions = new HashSet<ReactionInstance>();

    /**
     * Map from reactor instances to a map from reaction to reaction instances.
     */
    private HashMap<ReactorInstance, LinkedHashMap<Reaction, ReactionInstance>> _reactorToReactionList = new HashMap<ReactorInstance, LinkedHashMap<Reaction, ReactionInstance>>();
    // FIXME: move this to ReactorInstance
    //////////////////////////////////////////////////////////////////////////
    //// Inner classes

    /**
     * Inner class for each reaction instance.
     */
    public class ReactionInstance {

        public ReactionInstance(Reaction reactionSpec,
                ReactorInstance reactorInstance) {
            this.reactionSpec = reactionSpec;
            this.reactorInstance = reactorInstance;
        }

        /** The ports that this reaction may write to. */
        public HashSet<PortInstance> dependentPorts = new HashSet<PortInstance>();

        /** The ports that this reaction is triggered by or uses. */
        public HashSet<PortInstance> dependsOnPorts = new HashSet<PortInstance>();

        /** The reactions that depend on this reaction. */
        public HashSet<ReactionInstance> dependentReactions = new HashSet<ReactionInstance>();

        /** The reactions that this reaction depends on. */
        public HashSet<ReactionInstance> dependsOnReactions = new HashSet<ReactionInstance>();

        /** The level in the dependence graph. */
        public int level = 0;

        /** The AST node specifying the reaction. */
        public Reaction reactionSpec;

        /** The instance of the reactor. */
        public ReactorInstance reactorInstance;

        /**
         * Place to store properties specific to a particular code generator.
         */
        public HashMap<String, String> properties = new HashMap<String, String>();

        /**
         * Add to the dependsOnReactions and dependentReactions all the
         * reactions that this reaction depends on indirectly through ports or
         * that depend on this reaction. Clear out the dependentPorts and
         * dependsOnPorts sets. If there are ultimately no reactions that this
         * reaction depends on, then add this reaction to the list of
         * independent reactions.
         */
        public void collapseDependencies() {
            HashSet<PortInstance> visited = new HashSet<PortInstance>();
            for (PortInstance port : dependentPorts) {
                port.collapseDependencies(visited);
                dependentReactions.addAll(port.dependentReactions);
            }
            dependentPorts.clear();
            for (PortInstance port : dependsOnPorts) {
                port.collapseDependencies(visited);
                dependsOnReactions.addAll(port.dependsOnReactions);
            }
            dependsOnPorts.clear();
            if (dependsOnReactions.isEmpty()) {
                _independentReactions.add(this);
            }
        }
    }
}
