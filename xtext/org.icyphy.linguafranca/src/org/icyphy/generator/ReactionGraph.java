package org.icyphy.generator;


import java.util.HashSet;
import java.util.LinkedHashSet;
import java.util.Set;

import org.icyphy.linguaFranca.Reaction;

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
     * Recursively collect reactions contained in a given reactor instance
     * and all of the reactors that it contains.
     * 
     * @param reactorInstance The reactor instance.
     */
    private void collectNodes(ReactorInstance reactorInstance) {
        // Add all reactions of the given reactor instance as nodes of the graph.
        nodes.addAll(reactorInstance.reactionInstances.values());
        // Do the same for all of its children
        for (ReactorInstance containedReactor : reactorInstance.children) {
            collectNodes(containedReactor);
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
        // Collect the nodes of the dependence graph.
        collectNodes(main);
        // Collapse the dependence graph to only include reactions (not ports).
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
                            .append(instance.parent.getFullName());
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
            collapseDependencies(reaction);
        }
    }

    /** Return the reaction instance in the specified reactor instance
     *  that corresponds to the specified reaction definition.
     *  @param reactorInstance The reactor instance for the reaction.
     *  @param reaction The reaction specification in the AST.
     *  @return The reaction instance or null if there is none.
     */
    public ReactionInstance getReactionInstance(ReactorInstance reactorInstance, // FIXME: We don't use this. Remove?
            Reaction reaction) throws Exception {
        return reactorInstance.reactionInstances.get(reaction);
    }

    //////////////////////////////////////////////////////////////////////////
    //// Private fields

    /** The code generator using this graph. */
    private GeneratorBase _generator;

    /** Set of independent reactions. */
    private Set<ReactionInstance> _independentReactions = new HashSet<ReactionInstance>();

    //////////////////////////////////////////////////////////////////////////
    //// Inner classes

    /**
     * Add to the dependsOnReactions and dependentReactions all the
     * reactions that this reaction depends on indirectly through ports or
     * that depend on this reaction. Clear out the dependentPorts and
     * dependsOnPorts sets. If there are ultimately no reactions that this
     * reaction depends on, then add this reaction to the list of
     * independent reactions.
     */
    protected void collapseDependencies(ReactionInstance reactionInstance) {
        //HashSet<ReactionInstance> visited = new HashSet<ReactionInstance>();
        for (PortInstance port : reactionInstance.dependentPorts) {
            //port.collapseDependencies(visited);
            reactionInstance.dependentReactions.addAll(port.dependentReactions);
        }
        reactionInstance.dependentPorts.clear(); // FIXME: why are we clearing these?
        for (PortInstance port : reactionInstance.dependsOnPorts) {
            //port.collapseDependencies(visited);
            reactionInstance.dependsOnReactions.addAll(port.dependsOnReactions);
        }
        reactionInstance.dependsOnPorts.clear(); // FIXME: why are we clearing these?
        if (reactionInstance.dependsOnReactions.isEmpty()) {
            _independentReactions.add(reactionInstance);
        }
    }
    
}
