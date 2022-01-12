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
import org.lflang.CausalityInfo
import org.lflang.TimeUnit
import org.lflang.TimeValue
import java.util.Arrays
import java.util.HashMap
import java.util.HashSet
import java.util.LinkedList
import java.util.List
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
 class CausalityMap {

    // The main reactor instance that this graph is associated with
    var ReactorInstance main

    // The reaction upon which the causality graph is built
    var ReactionInstanceGraph reactionGraph
    
    // Adjacency map from a pair of reactions (i.e. components)
    // to a list of causal relations.
    // 
    // FIXME: We need to detect common triggers for different reactions
    // (from a trigger's perspective) as well as the set of sourses
    // for a reaction (from a reaction's perspective). The latter can be
    // addressed by the ReactionInstance class. Perhaps we need a trigger map
    // where keys are triggers and values are reactions.
    // Currently the former task is done by iterating the causality map.
    // The awkwardness shows up in "Reactions and Their Triggers."
    public var HashMap<Pair<ReactionInstance, ReactionInstance>,
        List<CausalityInfo>> causality = new HashMap
    
    // The set of ports
    public var Set<PortInstance> ports = new HashSet

    // The set of actions
    public var Set<ActionInstance> actions = new HashSet

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
        // FIXME: Can we just pass in one source instead of iterating
        // over all the nodes?
        for (node : this.reactionGraph.nodes) {
            traverseGraph(node)
        }
        // After building the graph, check if there are any dangling ports.
        // This is currently done here when all the connections are detected.
        for (node : this.reactionGraph.nodes) {
            checkDanglingPorts(node)
        }
    }
    
    /**
     * @brief Build the graph by adding nodes and edges based on the reaction graph.
     *
     * @param reaction The upstream reaction instance to be added to the graph.
     */
    protected def void traverseGraph(ReactionInstance reaction) {
        // Check for startup triggers
        // FIXME: Handle shutdown
        for (t : reaction.triggers) {
            if (t.isStartup) {
                var key = new Pair(reaction, reaction)
                var info = new CausalityInfo(
                    "startup",  // type
                    t,          // triggerInstance
                    false,      // isPhysical
                    0,          // delay
                    null,       // upstreamPort
                    null)       // downstreamPort
                println("startup triggers detected for " + reaction)
                println(key)
                println(info)
                addKeyInfoToCausalityMap(key, info)
            }
            else if (t instanceof TimerInstance) {
                var key = new Pair(reaction, reaction)
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
            if (e instanceof PortInstance) this.ports.add(e)
            else if (e instanceof ActionInstance) this.actions.add(e)
        }
        for (s : upstreamSources) {
            if (s instanceof PortInstance) this.ports.add(s)
            else if (s instanceof ActionInstance) this.actions.add(s)
        }
        
        // Check for self-triggering actions
        for (a : this.actions) {
            if (upstreamSources.contains(a) && upstreamEffects.contains(a)) {
                // Add the delay info to the causality hashmap.
                var key = new Pair(reaction, reaction)
                var info = new CausalityInfo(
                    "action",                  // type
                    a,                         // triggerInstance
                    a.isPhysical,              // isPhysical
                    a.minDelay.toNanoSeconds,  // delay
                    null,                      // upstreamPort
                    null)                      // downstreamPort
                addKeyInfoToCausalityMap(key, info)
            }
        }
            
        // Recursively add downstream nodes.
        var downstreamAdjNodes = this.reactionGraph.getDownstreamAdjacentNodes(reaction)
        for (downstream : downstreamAdjNodes) {            
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
                            // This does not generate a connection axiom.
                            var info = new CausalityInfo(
                                "barrier",  // type
                                ds,         // triggerInstance
                                false,      // isPhysical
                                0,          // delay
                                null,       // upstreamPort
                                null)       // downstreamPort
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
                            if (connection !== null) {
                                var long nanoSec = 0
                                if (connection.getDelay !== null) {
                                    var delayInterval = connection.getDelay.getTime.getInterval
                                    var TimeUnit delayUnit = TimeUnit.fromName(connection.getDelay.getTime.getUnit)
                                    var TimeValue timeValue = new TimeValue(delayInterval, delayUnit)
                                    nanoSec = timeValue.toNanoSeconds
                                }
                                var info = new CausalityInfo(
                                    "connection",           // type
                                    null,                   // triggerInstance
                                    connection.isPhysical,  // isPhysical
                                    nanoSec,                // delay
                                    ue as PortInstance,     // upstreamPort
                                    ds as PortInstance)     // downstreamPort
                                addKeyInfoToCausalityMap(key, info)
                            }
                        }
                    }
                }
            }
            // Recursively add downstream nodes to the graph.
            traverseGraph(downstream)
        }
    }

    /**
     * @brief Check dangling ports by traversing the graph.
     *
     * @param reaction The upstream reaction instance to be added to the graph.
     */
    protected def void checkDanglingPorts(ReactionInstance reaction) {
        var upstreamSources = reaction.sources
        for (source : reaction.sources) {
            if (source instanceof PortInstance) {
                // Check if this port is used.
                // Loop through entries where this reaction is being triggered.
                var used = false
                for (entry : this.causality.entrySet.filter[it.getKey.getValue.getFullName == reaction.getFullName]) {
                    for (info : entry.getValue) {
                        if (info.downstreamPort == source) used = true;
                    }
                }
                if (!used) {
                    println("Detect dangling port: " + source)
                    // Add this dangling port to the map.
                    var key = new Pair(reaction, reaction)
                    var info = new CausalityInfo(
                        "dangling_port", // type
                        source,          // triggerInstance
                        false,           // isPhysical
                        0,               // delay
                        null,            // upstreamPort
                        source)          // downstreamPort
                    addKeyInfoToCausalityMap(key, info)
                }
            }
        }
        // Recursively add downstream nodes.
        var downstreamAdjNodes = this.reactionGraph.getDownstreamAdjacentNodes(reaction)
        for (downstream : downstreamAdjNodes) {
            // Recursively add downstream nodes to the graph.
            traverseGraph(downstream)
        }
    }

    protected def void addKeyInfoToCausalityMap(Pair<ReactionInstance, ReactionInstance> key, CausalityInfo info) {
        if (causality.get(key) === null) {
            causality.put(key, new LinkedList<CausalityInfo>(Arrays.asList(info)))
        } else {
            // Add to the list if it is not redundant.
            if (causality.get(key).filter[it.type == info.type
                && it.triggerInstance == info.triggerInstance].length == 0) {
                causality.get(key).add(info)
            }
        }
    }

    def void printInfo(Pair<ReactionInstance, ReactionInstance> key) {
        var list = causality.get(key)
        println('''
        Connectivity info for «key.getKey» and «key.getValue»
        ''')
        for (var i = 0; i < list.size; i++) {
            println('''
            --------------------------
            Index: «i»
            Type: «list.get(i).type»
            IsPhysical: «list.get(i).isPhysical»
            Delay (ns): «list.get(i).delay»
            ''')
        }
    }

    def void printInfo(Pair<ReactionInstance, ReactionInstance> key, CausalityInfo info) {
        println('''
        Connectivity info
        -------------------------
        Upstream: «key.getKey»
        Downstream: «key.getValue»
        Type: «info.type»
        IsPhysical: «info.isPhysical»
        Delay (ns): «info.delay»
        ''')
    }
 }