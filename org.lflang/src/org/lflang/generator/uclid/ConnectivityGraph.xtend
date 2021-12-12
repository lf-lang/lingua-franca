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
import org.lflang.ConnectivityInfo
import java.util.HashMap
import java.util.HashSet
import java.util.Set

/**
 * This graph represents the connectivity between system components,
 * which indicates counterfactual causality between events generated
 * by connected components.
 *
 * TODO:
 * 1. Handle timers.
 * 
 * @author{Shaokai Lin <shaokai@eecs.berkeley.edu>}
 */
 class ConnectivityGraph extends DirectedGraph<ReactionInstance> {

    // The main reactor instance that this graph is associated with
    var ReactorInstance main

    // The reaction upon which the connectivity graph is built
    var ReactionInstanceGraph reactionGraph
    
    // Adjacency map from a pair of reactions (i.e. components)
    // to a pair (to be extended to triplet) containing a boolean
    // (whether it is a connection) and a TimeValue (logical time delay).
    public var HashMap<Pair<ReactionInstance, ReactionInstance>,
        ConnectivityInfo> connectivity = new HashMap();
    
    // The set of ports
    public var Set<PortInstance> ports = new HashSet();

    // Constructor
    new(ReactorInstance main, ReactionInstanceGraph reactionGraph) {
        this.main = main
        this.reactionGraph = reactionGraph
        rebuild()
    }
    
    /**
     * Rebuild this graph by first clearing and then re-extracting
     * info from the reaction graph.
     */
    def rebuild() {
        this.clear()
        println("========== Building Connectivity Graph ==========")
        // FIXME: Can we just pass in one source instead of iterating
        // over all the nodes?
        for (node : this.reactionGraph.nodes) {
            addNodesAndEdges(node)
        }
        println("Ports: " + this.ports)
    }
    
    /**
     * @brief Build the graph by adding nodes and edges based on the reaction graph.
     * 
     * FIXME:
     * 1. Do we need transitive edges?
     *    No. Handle transitivity in the code generation logic.
     *
     * @param reaction The upstream reaction instance to be added to the graph.
     */
    protected def void addNodesAndEdges(ReactionInstance reaction) {
        // Add the current reaction to the connectivity graph.
        this.addNode(reaction)
        
        // Collect port instances that can influence the upstream reaction
        // Note: The dangling ports are ignored in this case.
        var upstreamSources = reaction.sources
        var upstreamEffects = reaction.effects
        for (e : upstreamEffects) {
            if (e instanceof PortInstance) ports.add(e)
        }
        for (s : upstreamSources) {
            if (s instanceof PortInstance) ports.add(s)
        }
            
        // Recursively add downstream nodes.
        println("Reaction " + reaction + " has the following downstream nodes.")
        var downstreamAdjNodes = this.reactionGraph.getDownstreamAdjacentNodes(reaction)
        for (downstream : downstreamAdjNodes) {            
            // Add an edge
            println(downstream)
            this.addEdge(downstream, reaction)
            
            // Store logical delay between the two reactions.
            // Logical delays can be induced by connections
            // or actions.
            var downstreamSources = downstream.sources      // Sources of the downstream element
            var key     = new Pair(reaction, downstream)            
            for (ue : upstreamEffects) {
                for (ds : downstreamSources) {
                    // If these two reactions are linked by an action,
                    // add the corresponding connectivity info to the graph.
                    if (ds == ue) {
                        if (ds instanceof ActionInstance) {
                            // Add the delay info to the connectivity hashmap.
                            var info = new ConnectivityInfo(false, ds.isPhysical, ds.minDelay.toNanoSeconds, null, null)
                            if (connectivity.get(key) === null) {
                                connectivity.put(key, info)
                                println("New connectivity info added")
                                printInfo(key)
                            }
                        }
                        else {
                            throw new RuntimeException('Unhandled case: The source is equal to the effect, but it is not an ActionInstance.')
                        }
                    }
                    else {
                        if (ds instanceof PortInstance && ue instanceof PortInstance) {
                            var connection = main.getConnection(ue as PortInstance, ds as PortInstance)
                            println("connection: " + connection)
                            if (connection !== null) {
                                // FIXME: what is the relationship between delay and TimeValue.                                
                                // FIXME: get delay
                                var info = new ConnectivityInfo(true, connection.isPhysical, 0, ds as PortInstance, ue as PortInstance)
                                // connectivity.put(new Pair(s, e), new Pair(true, connection.getDelay.getInterval))
                                if (connectivity.get(key) === null) {
                                    connectivity.put(key, info)
                                    println("New connectivity info added")
                                    printInfo(key)
                                }
                            }
                        }
                    }
                }
            }

            // Recursively add downstream nodes to the graph.
            addNodesAndEdges(downstream)
        }
    }

    protected def void printInfo(Pair<ReactionInstance, ReactionInstance> key) {
        var info = connectivity.get(key)
        println("Connectivity info:\n"
            + "------------------------------\n"
            + "upstream: " + key.getKey + "\n"
            + "downstream: " + key.getValue + "\n"
            + "isConnection: " + info.isConnection + "\n"
            + "isPhysical: " + info.isPhysical + "\n"
            + "delay: " + info.delay)
    }
 }