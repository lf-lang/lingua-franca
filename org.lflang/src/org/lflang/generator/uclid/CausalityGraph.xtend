/** A graph that represents the causality between system components. */

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

import org.lflang.generator.TimerInstance
import org.lflang.graph.DirectedGraph
import org.lflang.CausalityInfo
import org.lflang.TimeUnit
import org.lflang.TimeValue
import java.util.HashMap
import java.util.HashSet
import java.util.Set

/**
 * This graph represents the causality between system components,
 * which indicates counterfactual causality between events generated
 * by connected components.
 *
 * TODO:
 * 1. Handle timers.
 * 
 * @author{Shaokai Lin <shaokai@eecs.berkeley.edu>}
 */
 class CausalityGraph extends DirectedGraph<ReactionInstance> {

    // The main reactor instance that this graph is associated with
    var ReactorInstance main

    // The reaction upon which the causality graph is built
    var ReactionInstanceGraph reactionGraph
    
    // Adjacency map from a pair of reactions (i.e. components)
    // to a pair (to be extended to triplet) containing a boolean
    // (whether it is a connection) and a TimeValue (logical time delay).
    public var HashMap<Pair<ReactionInstance, ReactionInstance>,
        CausalityInfo> causality = new HashMap();
    
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
     * @param reaction The upstream reaction instance to be added to the graph.
     */
    protected def void addNodesAndEdges(ReactionInstance reaction) {
        // Add the current reaction to the causality graph.
        this.addNode(reaction)

        // Check for startup triggers
        // FIXME: Handle shutdown
        for (t : reaction.triggers) {
            if (t.isStartup) {
                var key = new Pair(reaction, null)
                var info = new CausalityInfo(
                    "startup",  // type
                    t,          // triggerInstance
                    false,      // isPhysical
                    0,          // delay
                    null,       // upstreamPort
                    null)       // downstreamPort
                addKeyInfoToCausalityMap(key, info)
            }
            else if (t instanceof TimerInstance) {
                var key = new Pair(reaction, null)
                var info = new CausalityInfo(
                    "timer",    // type
                    t,          // triggerInstance
                    false,      // isPhysical
                    0,          // delay
                    null,       // upstreamPort
                    null)       // downstreamPort
                addKeyInfoToCausalityMap(key, info)
            }
        }
        
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
            var key = new Pair(reaction, downstream)            
            for (ue : upstreamEffects) {
                for (ds : downstreamSources) {
                    // If these two reactions are linked by an action,
                    // add the corresponding causality info to the graph.
                    if (ds == ue) {
                        if (ds instanceof ActionInstance) {
                            // Add the delay info to the causality hashmap.
                            var info = new CausalityInfo(
                                "action",                   // type
                                ds,                         // triggerInstance
                                ds.isPhysical,              // isPhysical
                                ds.minDelay.toNanoSeconds,  // delay
                                null,                       // upstreamPort
                                null)                       // downstreamPort
                            addKeyInfoToCausalityMap(key, info)
                        }
                        else if (ds instanceof PortInstance) {
                            // This case happens when a reaction in the main reactor
                            // triggers a reaction in the nested reactor.
                            // Can be interpreted as having a connection of 0 delay.
                            // The upstream and downstream port instances are the same.
                            var info = new CausalityInfo(
                                "connection",   // type
                                null,           // triggerInstance
                                false,          // isPhysical
                                0,              // delay
                                ds,             // upstreamPort
                                ds)             // downstreamPort
                            addKeyInfoToCausalityMap(key, info)
                        }
                        else {
                            throw new RuntimeException('Unhandled case: The source is equal to the effect, 
                                but it is not an ActionInstance, nor a PortInstance: ' + ds)
                        }
                    }
                    else {
                        if (ds instanceof PortInstance && ue instanceof PortInstance) {
                            var connection = this.main.getConnection(ue as PortInstance, ds as PortInstance)
                            println("connection: " + connection)
                            if (connection !== null) {
                                var delayInterval = connection.getDelay.getTime.getInterval
                                var TimeUnit delayUnit = TimeUnit.fromName(connection.getDelay.getTime.getUnit)
                                var TimeValue timeValue = new TimeValue(delayInterval, delayUnit)
                                var info = new CausalityInfo(
                                    "connection",               // type
                                    null,                       // triggerInstance
                                    connection.isPhysical,      // isPhysical
                                    timeValue.toNanoSeconds,    // delay
                                    ue as PortInstance,         // upstreamPort
                                    ds as PortInstance)         // downstreamPort
                                addKeyInfoToCausalityMap(key, info)
                            }
                        }
                    }
                }
            }

            // Recursively add downstream nodes to the graph.
            addNodesAndEdges(downstream)
        }
    }

    protected def void addKeyInfoToCausalityMap(Pair key, CausalityInfo info) {
        if (causality.get(key) === null) {
            causality.put(key, info)
            println("New causality info added:")
            printInfo(key)
        }
        else {
            println("Key-info pair was not added. Causality info already exists:")
            printInfo(key)
        }
    }

    protected def void printInfo(Pair<ReactionInstance, ReactionInstance> key) {
        var info = causality.get(key)
        println("Connectivity info:\n"
            + "------------------------------\n"
            + "upstream: " + key.getKey + "\n"
            + "downstream: " + key.getValue + "\n"
            + "type: " + info.type + "\n"
            + "isPhysical: " + info.isPhysical + "\n"
            + "delay (ns): " + info.delay)
    }
 }