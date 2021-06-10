/* Dependency graph of a reactor program. */

/*************
 * Copyright (c) 2020, The University of California at Berkeley.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/
package org.lflang.graph

import org.lflang.generator.NamedInstance
import org.lflang.generator.PortInstance
import org.lflang.generator.ReactionInstance
import org.lflang.generator.ReactorInstance

/**
 * A graph with vertices that are ports or reactions and edges that denote
 * dependencies between them.
 * 
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class TopologyGraph extends PrecedenceGraph<NamedInstance<?>> {

    /**
     * Construct a reaction graph based on the given reactor instances and run Tarjan's
     * algorithm to detect cyclic dependencies between reactions. It is
     * assumed that no instantiation cycles are present in the program.
     * Checks for instantiation cycles thus must be carried out prior to  
     * constructing this graph.
     * @param model The reactor instances to construct the graph for.
     */
    new(ReactorInstance ...reactors) {
        for (r : reactors) {
            collectNodesFrom(r)    
        }
        this.detectCycles()
    }

    /**
     * Traverse the subtree that is a reactor definition that is bound to a particular series of instantiations.
     * 
     * @param node A bound reactor class instantiation.
     */
    def void collectNodesFrom(ReactorInstance reactor) {
        var ReactionInstance previousReaction = null
        for (reaction : reactor.reactions) {
            this.addNode(reaction)
            
            reaction.effects.filter(PortInstance).forEach [
                addEdge(it, reaction)
                it.dependentPorts.forEach[sink|addEdge(sink, it)]
            ]

            reaction.sources.filter(PortInstance).forEach [
                addEdge(reaction, it)
                for (upstream : it.dependsOnPorts) {
                    addEdge(it, upstream)
                }
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
            collectNodesFrom(child)
        }
    }
}
