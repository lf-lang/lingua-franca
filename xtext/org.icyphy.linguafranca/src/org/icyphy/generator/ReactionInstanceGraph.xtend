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

package org.icyphy.generator

import org.icyphy.graph.DirectedGraph

/**
 * This graph is part of the code generation package because it is not generic.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class ReactionInstanceGraph extends DirectedGraph<ReactionInstance> {
    
    /**
     * The main reactor instance that this graph is associated with.
     */
    var ReactorInstance main
    
    /**
     * Create a new graph by traversing the maps in the named instances 
     * embedded in the hierarchy of the program. 
     */
    new(ReactorInstance main) {
        this.main = main
        addNodesAndEdges(main)
    }
    
    /**
     * Rebuild this graph by clearing is an repeating the traversal that 
     * adds all the nodes and edges.
     */
    def rebuild() {
        this.clear()
        addNodesAndEdges(main)
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
            reaction.effects.forEach [ effect |
                addDownstreamReactions(effect, reaction)
            ]
            
            // Reactions that write to such a port are also reactions that
            // that this reaction depends on, by transitivity.
            reaction.sources.forEach [ source |
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
