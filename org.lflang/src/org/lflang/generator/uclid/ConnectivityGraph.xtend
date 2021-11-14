/** A graph that represents the connectivity between system components. */

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

import org.lflang.graph.DirectedGraph
import java.util.LinkedHashMap
import org.lflang.ConnectivityInfo

/**
 * This graph represents the connectivity between system components,
 * which indicates counterfactual causality between events generated
 * by connected components.
 * 
 * @author{Shaokai Lin <shaokai@berkeley.edu>}
 */
 class ConnectivityGraph extends DirectedGraph<ReactionInstance> {
     
    /**
     * The main reactor instance that this graph is associated with.
     */
    var ReactorInstance main
    
    var ReactionInstanceGraph reactionGraph
    
    /**
     * Adjacency map from a pair of reactions (i.e. components)
     * to a pair (to be extended to triplet) containing a boolean
     * (whether it is a connection) and a TimeValue (logical time delay).
     */
    var LinkedHashMap<Pair<ReactionInstance, ReactionInstance>,
        ConnectivityInfo> connectivity = new LinkedHashMap();
    
    new(ReactorInstance main) {
        this.main = main
        this.reactionGraph = new ReactionInstanceGraph(main)
        rebuild()
    }
    
    /**
     * Rebuild this graph by first clearing and then re-extracting
     * info from the reaction graph (for now (We will build extensions
     * on top of the reaction graph to capture more components later.)).
     */
    def rebuild() {
        this.clear()
        
        println("========== Building Connectivity Graph ==========")
        for (node : this.reactionGraph.nodes) {
            addNodesAndEdges(node)
        }
    }
    
    /**
     * Build the graph by adding nodes and edges based on the reaction graph.
     * 
     * FIXME:
     * 1. Add nodes.
     * 2. Add edges based on connection map in the reactor instance.
     */
    protected def void addNodesAndEdges(ReactionInstance reaction) {
        // Add the current reaction to the connectivity graph.
        this.addNode(reaction)
            
        // Recursively add downstream nodes.
        println("Reaction " + reaction + " has the following downstream nodes.")
        var downstreamAdjNodes = this.reactionGraph.getDownstreamAdjacentNodes(reaction)
        for (node : downstreamAdjNodes) {
            println(node)
            
            // Recursively add downstream nodes to the graph.
            addNodesAndEdges(node)
            
            // Add an edge
            this.addEdge(node, reaction)
            
            // Store logical delay between the two reactions.
            // Logical delays can be induced by connections
            // or actions.
            var effects = reaction.effects
            var sources = node.sources
            var key     = new Pair(reaction, node)
            var info    = new ConnectivityInfo(false, false, 0)
            
            for (e : effects) {
                for (s : sources) {
                    // If these two reactions are linked by an action.
                    if (s == e) {
                        if (s instanceof ActionInstance) {
                            // Add the delay info to the connectivity hashmap.
                            info = new ConnectivityInfo(false, s.isPhysical, s.minDelay.toNanoSeconds)
                            
                            if (connectivity.get(key) === null) {
                                connectivity.put(key, info)
                                println("New connectivity info added. isConnection: " + info.isConnection
                                    + ", isPhysical: " + info.isPhysical + ", delay: " + info.delay)
                            }
                        }
                        else {
                            // FIXME: this should not happen.
                        }
                    }
                    else {
                        if (s instanceof PortInstance && e instanceof PortInstance) {
                            var connection = main.getConnection(e as PortInstance, s as PortInstance)
                            println(s)
                            println(e)
                            println("connection: " + connection)
                            if (connection !== null) {
                                // FIXME: how to break break for loops in xtend?
                                // FIXME: what is the relationship between delay and TimeValue.
                                // Add the delay info to the connectivity hashmap.
                                // FIXME: is there a better way to get int value?
                                
                                // FIXME: get delay
                                info = new ConnectivityInfo(true, connection.isPhysical, 0)
                                // connectivity.put(new Pair(s, e), new Pair(true, connection.getDelay.getInterval))
                                if (connectivity.get(key) === null) {
                                    connectivity.put(key, info)
                                    println("New connectivity info added. isConnection: " + info.isConnection
                                        + ", isPhysical: " + info.isPhysical + ", delay: " + info.delay)                                }
                            }
                        }
                    }
                }
            }
        }
    }
 }