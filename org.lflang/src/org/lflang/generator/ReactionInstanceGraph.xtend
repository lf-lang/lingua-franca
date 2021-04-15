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

package org.lflang.generator

import java.util.ArrayList
import java.util.LinkedHashSet
import java.util.Set
import org.lflang.graph.DirectedGraph

/**
 * This graph represents the dependencies between reaction instances.
 * Upon creation, reactions are assigned levels, chainIDs, and their
 * deadlines are propagated.
 * 
 * This class is distinct from the ReactionGraph class in the graph
 * package because it deals with instance objects rather than AST nodes.
 * 
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 * @author{Edward A. Lee <eal@berkeley.edu>}
 */
class ReactionInstanceGraph extends DirectedGraph<ReactionInstance> {
    
    /**
     * The main reactor instance that this graph is associated with.
     */
    var ReactorInstance main
    
    /**
     * Count of the number of chains while assigning chainIDs.
     */
    int branchCount = 1
    
    /**
     * Create a new graph by traversing the maps in the named instances 
     * embedded in the hierarchy of the program. 
     */
    new(ReactorInstance main) {
        this.main = main
        rebuild()
    }
    
    /**
     * Rebuild this graph by clearing is an repeating the traversal that 
     * adds all the nodes and edges.
     */
    def rebuild() {
        this.clear()
        addNodesAndEdges(main)
            // Assign a level to each reaction. 
            // If there are cycles present in the graph, it will be detected here.
            if (!assignLevels()) {
                main.generator.reportError(main.generator.mainDef, "Reactions form a cycle!");
                throw new Exception("Reactions form a cycle!")
            }
            // Traverse the graph again, now starting from the leaves,
            // to set the chain IDs.
            assignChainIDs(true)

            // Propagate any declared deadline upstream.
        propagateDeadlines()

    }

    /**
     * Return the single dominating reaction if the given reaction has one, or
     * null otherwise.
     */
    def findSingleDominatingReaction(ReactionInstance reaction) {
        this.nodes.forEach[node|node.visitsLeft = 1]
        val reactions = reaction.upstreamAdjacentNodes
        if (reactions.size > 0) {
            val minLevel = reactions.fold(newLinkedList, [ list, r |
                list.add(r.level)
                return list
            ]).min
            val maximalReactions = maximal(reactions.toSet, minLevel)
            if (maximalReactions.size == 1) {
                return maximalReactions.get(0)
            }
        }
        return null
    }

    /**
     * From the given set of reactions, return the subset that is maximal. 
     * A reaction in the set is maximal if there is no other reaction in 
     * the set that depends on it, directly or indirectly. If the argument 
     * is an empty set, return an empty set.
     * @param reactions A set of reaction instances.
     * @param minLevel The lowest level reaction to visit.
     */
    protected def Set<ReactionInstance> maximal(Set<ReactionInstance> reactions,
        long minLevel) {
        var result = new LinkedHashSet(reactions)
        for (reaction : reactions) {
            if (reaction.level >= minLevel && reaction.visitsLeft > 0) {
                reaction.visitsLeft--
                val upstream = getUpstreamAdjacentNodes(reaction)
                result.removeAll(upstream)
                result.removeAll(maximal(upstream.toSet, minLevel))
            }
        }
        return result
    }
    
    /**
     * Analyze the dependencies between reactions and assign each reaction
     * instance a level.
     * This procedure is based on Kahn's algorithm for topological sorting.
     * Rather than establishing a total order, we establish a partial order.
     * In this order, the level of each reaction is the least upper bound of
     * the levels of the reactions it depends on.
     * If any cycles are present in the dependency graph, an exception is
     * thrown. This method should be called only on the top-level (main) reactor.
     * @return true if the assignment was successful, false if it was not, 
     * meaning the graph has at least one cycle in it.
     */
    protected def assignLevels() {
        val graph = this.copy
        var start = new ArrayList(graph.rootNodes)
        
        // All root nodes start with level 0.
        for (origin : start) {
            origin.level = 0
        }

        // No need to do any of this if there are no root nodes; 
        // the graph must be cyclic.
        if (!graph.rootNodes.isEmpty) {
            while (!start.empty) {
                val origin = start.remove(0)
                val toRemove = new LinkedHashSet<ReactionInstance>()
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
        if (graph.nodeCount != 0) {
            return false
        }

        return true
    }
    
    /**
     * Analyze the dependencies between reactions and assign each reaction
     * instance a chain identifier. The assigned IDs are such that the
     * bitwise conjunction between two chain IDs is always nonzero if there
     * exists a dependency between them. This facilitates runtime checks
     * to determine whether a reaction is ready to execute or has to wait
     * for an upstream reaction to complete.
     * @param graph The dependency graph.
     * @param optimize Whether or not make assignments that maximize the
     * amount of parallelism. If false, just assign 1 to every node.
     */
    protected def assignChainIDs(boolean optimize) {
        val leafs = this.leafNodes
        this.branchCount = 0
        for (node : this.nodes) {
            node.visitsLeft = this.getDownstreamAdjacentNodes(node).size
        }
        if (optimize) {
            // Start propagation from the leaf nodes,
            // ordered by level from high to low.
            for (node : leafs.sortBy[-level]) {
                this.propagateUp(node, 1 << (this.branchCount++ % 64))
            }    
        } else {
            for (node: this.nodes) {
            node.chainID = 1
            }    
        }
    }
    
    /**
     * Propagate the given chain ID up one chain, propagate fresh IDs to
     * other upstream neighbors, and return a mask that overlaps with all the
     * chain IDs that were set upstream as a result of this method invocation.
     * The result of propagation is that each node has an ID that overlaps with
     * all upstream nodes that can reach it. This means that if a node has a
     * lower level than another node, but the two nodes do not have overlapping
     * chain IDs, the nodes are nonetheless independent from one another.
     * @param current The current node that is being visited.
     * @param graph The graph that encodes the dependencies between reactions.
     * @param chainID The current chain ID.
     */
    private def long propagateUp(ReactionInstance current, long chainID) {
        val origins = this.getUpstreamAdjacentNodes(current)
        var mask = chainID
        var first = true
        var id = current.chainID.bitwiseOr(chainID)
        current.visitsLeft--
        if (current.visitsLeft > 0) {
            current.chainID = id
            return chainID;
        }
        // Iterate over the upstream neighbors by level from high to low.
        for (upstream : origins.sortBy[-level]) {
            if (first) {
                // Stay on the same chain the first time.
                first = false
            } else {
                // Create a new chain ID.
                id = 1 << (this.branchCount++ % 64)
            }
            // Propagate the ID upstream and add all returned bits
            // to the mask.
            mask = mask.bitwiseOr(
                    propagateUp(upstream, id))
        }    
        // Apply the mask to the current chain ID.
        // If there were no upstream neighbors, the mask will
        // just be the chainID that was passed as an argument.
        current.chainID = current.chainID.bitwiseOr(mask)
        
        return mask
    }
    
    /**
     * Iterate over all reactions that have a declared deadline, update their
     * inferred deadline, as well as the inferred deadlines of any reactions
     * upstream.
     */
    def propagateDeadlines() {
        val reactionsWithDeadline = this.nodes.filter[it.definition.deadline !== null]
        // Assume the graph is acyclic.
        for (r : reactionsWithDeadline) {
            if (r.declaredDeadline !== null &&
                r.declaredDeadline.maxDelay !== null) {
                // Only lower the inferred deadline (which is set to the max by default),
                // if the declared deadline is earlier than the inferred one (based on
                // some other downstream deadline).
                if (r.deadline.isEarlierThan(r.declaredDeadline.maxDelay)) {
                    r.deadline = r.declaredDeadline.maxDelay
                }
            }
            propagateDeadline(r)
        }
    }
    
    /**
     * Given a reaction instance, propagate its inferred deadline upstream.
     * @param downstream Reaction instance with an inferred deadline that
     * is to be propagated upstream.
     */
    def void propagateDeadline(ReactionInstance downstream) {
        for (upstream : this.getUpstreamAdjacentNodes(downstream)) {
            // Only lower the inferred deadline (which is set to the max by default),
            // if downstream deadline is earlier than the inferred one (based on
            // some other downstream deadline).
            if (downstream.deadline.isEarlierThan(upstream.deadline)) {
                upstream.deadline = downstream.deadline
            }
            propagateDeadline(upstream)
        }
    }
    
    /**
     * Add to the graph edges between the given reaction and all the reactions
     * that the specified port depends on.
     * @param port The port that the given reaction as as a source.
     * @param reaction The reaction to relate upstream reactions to.
     */
    protected def void addUpstreamReactions(PortInstance port,
        ReactionInstance reaction) {
            // Reactions in the containing reactor.
        port.dependsOnReactions.forEach[this.addEdge(reaction, it)]
        // Reactions in upstream reactors.
        if (port.dependsOnPort !== null) {
            addUpstreamReactions(port.dependsOnPort, reaction)
        }
    }

    /**
     * Add to the graph edges between the given reaction and all the reactions
     * that depend on the specified port.
     * @param port The port that the given reaction as as an effect.
     * @param reaction The reaction to relate downstream reactions to.
     */
    protected def void addDownstreamReactions(PortInstance port,
        ReactionInstance reaction) {
            // Reactions in the containing reactor.
        port.dependentReactions.forEach[this.addEdge(it, reaction)]
        // Reactions in downstream reactors.
        for (downstreamPort : port.dependentPorts) {
            addDownstreamReactions(downstreamPort, reaction)
        }
    }

    
    /**
     * Build the graph by adding nodes and edges based on the given reactor 
     * instance.
     * @param reactor The reactor on the basis of which to add nodes and edges.
     */
    protected def void addNodesAndEdges(ReactorInstance reactor) {
        var ReactionInstance previousReaction = null
        for (reaction : reactor.reactions) {
            // Add reactions of this reactor.
            this.addNode(reaction)
            
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
        for (child : reactor.children) {
            addNodesAndEdges(child)
        }
    }
}
