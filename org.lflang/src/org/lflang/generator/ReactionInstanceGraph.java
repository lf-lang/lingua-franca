/** A graph that represents the dependencies between reaction instances. */

/*************
Copyright (c) 2021, The University of California at Berkeley.

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
import java.util.Set;
import org.lflang.graph.DirectedGraph;

/**
 * This graph represents the dependencies between reaction runtime instances.
 * For each ReactionInstance, there may be more than one runtime instance because
 * the ReactionInstance may be nested within one or more banks.
 * In the worst case, of these runtime instances may have distinct dependencies,
 * and hence distinct levels in the graph. Moreover, some of these instances
 * may be involved in cycles while others are not.
 * 
 * Upon creation, the runtime instances are created if necessary and stored
 * in the ReactionInstance.  These instances assigned levels (maximum number of
 * upstream reaction instances).
 * 
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Edward A. Lee <eal@berkeley.edu>}
 */
class ReactionInstanceGraph extends DirectedGraph<ReactionInstance.Runtime> {
    
    /**
     * Create a new graph by traversing the maps in the named instances 
     * embedded in the hierarchy of the program. 
     */
    public ReactionInstanceGraph(ReactorInstance main) {
        this.main = main;
        rebuild();
    }

    ///////////////////////////////////////////////////////////
    //// Public fields

    /**
     * The main reactor instance that this graph is associated with.
     */
    public final ReactorInstance main;
    
    ///////////////////////////////////////////////////////////
    //// Public methods
    
    /**
     * Return the single dominating reaction if the given reaction has one, or
     * null otherwise.
     */
    public ReactionInstance.Runtime findSingleDominatingReaction(ReactionInstance.Runtime r) {
        Set<ReactionInstance.Runtime> reactions = getUpstreamAdjacentNodes(r);
        if (reactions.size() == 1) {
            for(ReactionInstance.Runtime reaction : reactions) {
                return reaction;
            }
        }
        return null;
    }

    /**
     * Rebuild this graph by clearing and repeating the traversal that 
     * adds all the nodes and edges.
     */
    public void rebuild() {
        this.clear();
        addNodesAndEdges(main);
        // Assign a level to each reaction. 
        // If there are cycles present in the graph, it will be detected here.
        Set<ReactionInstance.Runtime> leftoverReactions = assignLevels();
        if (leftoverReactions.size() != 0) {
            // The validator should have caught cycles, but if there is a bug in some
            // AST transform such that it introduces cycles, then it is possible to have them
            // only detected here. An end user should never see this.
            main.reporter.reportError("Reactions form a cycle! " + leftoverReactions.toString());
            throw new InvalidSourceException("Reactions form a cycle!");
        }
    }
    
    ///////////////////////////////////////////////////////////
    //// Protected methods
        
    /**
     * Add to the graph edges between the given reaction and all the reactions
     * that depend on the specified port.
     * @param port The port that the given reaction as as an effect.
     * @param reaction The reaction to relate downstream reactions to.
     */
    protected void addDownstreamReactions(PortInstance port, ReactionInstance reaction) {
        // Reactions in the containing reactor.
        port.dependentReactions.forEach[this.addEdge(it, reaction)]
        // Reactions in downstream reactors.
        for (downstreamPort : port.dependentPorts) {
            addDownstreamReactions(downstreamPort.instance, reaction)
        }
    }

    /**
     * Build the graph by adding nodes and edges based on the given reactor 
     * instance.
     * @param reactor The reactor on the basis of which to add nodes and edges.
     */
    protected void addNodesAndEdges(ReactorInstance reactor) {
        ReactionInstance previousReaction = null;
        for (ReactionInstance reaction : reactor.reactions) {
            // Add reactions of this reactor.
            this.addNode(reaction);
            
            // Reactions that depend on a port that this reaction writes to
            // also, by transitivity, depend on this reaction instance.
            reaction.effects.filter(PortInstance).forEach [ effect |
                addDownstreamReactions(effect, reaction)
            ]
            
            // Reactions that write to such a port are also reactions that
            // that this reaction depends on, by transitivity.
            reaction.sources.filter(PortInstance).forEach [ source |
                addUpstreamReactions(source, reaction)
            ]
            // If this is not an unordered reaction, then create a dependency
            // on any previously defined reaction.
            if (!reaction.isUnordered) {
                // If there is an earlier reaction in this same reactor, then
                // create a link in the reaction graph.
                if (previousReaction !== null) {
                    this.addEdge(reaction, previousReaction)
                }
                previousReaction = reaction;
            }
        }
        // Recursively add nodes and edges from contained reactors.
        for (ReactorInstance child : reactor.children) {
            addNodesAndEdges(child);
        }
    }
    
    /**
     * Add to the graph edges between the given reaction and all the reactions
     * that the specified port depends on.
     * @param port The port that the given reaction as as a source.
     * @param reaction The reaction to relate upstream reactions to.
     */
    protected void addUpstreamReactions(PortInstance port, ReactionInstance.Runtime reaction) {
        // Reactions in the containing reactor.
        port.dependsOnReactions.forEach[this.addEdge(reaction, it)];
        // Reactions in upstream reactors.
        for (upstreamPort : port.dependsOnPorts) {
            addUpstreamReactions(upstreamPort.instance, reaction)
        }
    }

    ///////////////////////////////////////////////////////////
    //// Protected fields
    
    /**
     * Count of the number of chains while assigning chainIDs.
     */
    int branchCount = 1;
    
    ///////////////////////////////////////////////////////////
    //// Private methods

    /**
     * Analyze the dependencies between reactions and assign each reaction
     * instance a level.
     * This procedure is based on Kahn's algorithm for topological sorting.
     * Rather than establishing a total order, we establish a partial order.
     * In this order, the level of each reaction is the least upper bound of
     * the levels of the reactions it depends on.
     *
     * @return If any cycles are present in the dependency graph, then a graph
     * containing the nodes in the cycle is returned. Otherwise, null is
     * returned.
     */
    private Set<ReactionInstance.Runtime> assignLevels() {
        ReactionInstanceGraph graph = this.copy;
        List<ReactionInstance.Runtime> start = new ArrayList<ReactionInstance.Runtime>(graph.rootNodes());
        
        // All root nodes start with level 0.
        for (ReactionInstance.Runtime origin : start) {
            origin.level = 0;
        }

        // No need to do any of this if there are no root nodes; 
        // the graph must be cyclic.
        if (!graph.rootNodes.isEmpty) {
            while (!start.empty) {
                val origin = start.remove(0);
                val toRemove = new LinkedHashSet<ReactionInstance>();
                // Visit effect nodes.
                for (effect : graph.getDownstreamAdjacentNodes(origin)) {
                    // Stage edge between origin and effect for removal.
                    toRemove.add(effect)
                    
                    // Update level of downstream node.
                    effect.level = Math.max(effect.level, origin.level+1)    
                }
                // Remove visited edges.
                for (effect : toRemove) {
                    graph.removeEdge(effect, origin)
                    // If the effect node has no more incoming edges,
                    // then move it in the start set.
                    if (graph.getUpstreamAdjacentNodes(effect).size == 0) {
                        start.add(effect)
                    }
                }
                
                // Remove visited origin.
                graph.removeNode(origin)
                
            }
        }
        // If, after all of this, there are still any nodes left, 
        // then the graph must be cyclic.
        return graph;
    }
}
