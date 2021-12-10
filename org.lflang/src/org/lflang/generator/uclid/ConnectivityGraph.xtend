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
import java.util.LinkedHashMap
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
    var LinkedHashMap<Pair<ReactionInstance, ReactionInstance>,
        ConnectivityInfo> connectivity = new LinkedHashMap();
    
    // The set of ports
    // FIXME: why is this field not by default public?
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
            
        // Recursively add downstream nodes.
        println("Reaction " + reaction + " has the following downstream nodes.")
        var downstreamAdjNodes = this.reactionGraph.getDownstreamAdjacentNodes(reaction)
        for (downstream : downstreamAdjNodes) {
            println(downstream)
            
            // Add an edge
            this.addEdge(downstream, reaction)
            
            // Store logical delay between the two reactions.
            // Logical delays can be induced by connections
            // or actions.
            var effects = reaction.effects  // Effects of the upstream element
            var sources = downstream.sources      // Sources of the downstream element
            var key     = new Pair(reaction, downstream)            
            for (e : effects) {
                // Collect ports
                if (e instanceof PortInstance) ports.add(e)

                for (s : sources) {
                    // Collect ports
                    if (s instanceof PortInstance) ports.add(s)

                    // If these two reactions are linked by an action,
                    // add the corresponding connectivity info to the graph.
                    if (s == e) {
                        if (s instanceof ActionInstance) {
                            // Add the delay info to the connectivity hashmap.
                            var info = new ConnectivityInfo(reaction, downstream, false, s.isPhysical, s.minDelay.toNanoSeconds)
                            if (connectivity.get(key) === null) {
                                connectivity.put(key, info)
                                println("New connectivity info added")
                                printInfo(info)
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
                                var info = new ConnectivityInfo(reaction, downstream, true, connection.isPhysical, 0)
                                // connectivity.put(new Pair(s, e), new Pair(true, connection.getDelay.getInterval))
                                if (connectivity.get(key) === null) {
                                    connectivity.put(key, info)
                                    println("New connectivity info added")
                                    printInfo(info)
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

    protected def void printInfo(ConnectivityInfo info) {
        println("Connectivity info:\n"
            + "------------------------------\n"
            + "upstream: " + info.upstream + "\n"
            + "downstream: " + info.downstream + "\n"
            + "isConnection: " + info.isConnection + "\n"
            + "isPhysical: " + info.isPhysical + "\n"
            + "delay: " + info.delay)
    }
 }