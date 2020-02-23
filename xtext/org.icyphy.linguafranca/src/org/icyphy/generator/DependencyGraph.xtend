/* Instance of an action. */

/*************
Copyright (c) 2019, The University of California at Berkeley.

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

import java.util.HashMap
import java.util.Map
import java.util.Set
import java.util.HashSet

/** 
 * Directed graph that maps nodes to dependencies. 
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class DependencyGraph<T> {
    
    /**
     * Map vertices to the set of vertices that they depend on.
     */
    var Map<T, Set<T>> graph = new HashMap();
    
    /**
     * Construct a new dependency graph.
     */
    new() {
        
    }
    
    /**
     * Return as to whether this graph as the given node in it.
     * @param node The node to look for.
     */
    def hasNode(T node) {
        this.graph.get(node) === null ? false : true
    }
    
    /**
     * Return all the dependencies of a given node.
     * @param node The node to report the dependencies of.
     */
    def getDependencies(T node) {
        var ret = this.graph.get(node)
        if (ret === null) {
            return new HashSet()
        } else {
            return ret
        }
    }
    
    /**
     * Add the given node to the graph.
     * @param node The node to add to the graph.
     */
    def addNode(T node) {
        this.graph.put(node, new HashSet<T>())
    }
    
    /**
     * Remove the given node from the graph. This also eliminates any
     * outgoing and incoming edges of this node.
     * @param The node to remove
     */
    def removeNode(T node) {
        this.graph.remove(node)
        this.graph.forEach[v, e | e.remove(node)]
    }
    
    /**
     * Insert a new directed edge into the graph that flows from a
     * dependent to its dependency.
     * @param dependent The start point of the new directed edge.
     * @param dependency The end point of the new directed edge.
     */
    def addEdge(T dependent, T dependency) {
        // dependent -> dependency
        var deps = this.graph.get(dependent)
        if (deps === null) {
            deps = new HashSet()
            this.graph.put(dependent, deps)
        }
        deps.add(dependency)
        // Add the dependency as a node to the graph if its not there yet.
        if (!this.graph.containsKey(dependency)) {
            this.graph.put(dependency, new HashSet())    
        }
    }
    
    /**
     * Add new directed edges from a set of dependents to a single dependency.
     * @param dependency The end point of the new directed edges.
     * @param dependents The set of start points of the new directed edges.
     */
    def addBackEdges(T dependency, Set<T> dependents) {
        for (d : dependents) {
            this.addEdge(d, dependency)
        }
    }
    
    /**
     * Add new directed edges from a single dependent to a set of dependencies.
     * @param dependent The start point of the new directed edges.
     * @param dependencies The set of end points of the new directed edges.
     */
    def addEdges(T dependent, Set<T> dependencies) {
        for (d : dependencies) {
            this.addEdge(dependent, d)
        }
    }
    
    /**
     * Remove a directed edge from the graph.
     * @param dependent The start point of the edge to be removed.
     * @param dependency The end point of the edge to be removed.
     */
    def removeEdge(T dependent, T dependency) {
        var deps = this.graph.get(dependent)
        if (deps !== null && deps.contains(dependency)) {
            deps.remove(dependency)
        }
    }
    
    /**
     * Merge another dependency graph into this one.
     * @param another The graph to merge into this one.
     */
    def merge(DependencyGraph<T> another) {
        for (node : another.nodes) {
            var otherDeps = another.getDependencies(node)
            var theseDeps = this.graph.get(node)
            if (theseDeps === null) {
                // Node does not exist; add it.
                this.graph.put(node, otherDeps)
            } else {
                // Node does exist; add the missing dependencies.
                for (d : otherDeps) {
                    if (!theseDeps.contains(d)) {
                        theseDeps.add(d)
                    }
                }    
            }
        }
    }
    
    /**
     * Report the number of nodes in this graph.
     */
    def nodeCount() {
        this.graph.size
    }

    /**
     * Report the number of directed edges in this graph.
     */    
    def edgeCount() {
       var edges = 0
       for(node : nodes) {
           edges += this.graph.get(node).size
       }
       edges
    }
    
    /**
     * Return the nodes in this graph.
     */
    def nodes() {
        this.graph.keySet
    }
    
}