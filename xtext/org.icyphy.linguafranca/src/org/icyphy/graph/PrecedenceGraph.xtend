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

package org.icyphy.graph

import java.util.LinkedList
import java.util.List
import java.util.Stack
import java.util.Set
import java.util.HashSet
import java.util.HashMap

/** 
 * Elaboration of `DirectedGraph` that stores nodes in a wrapper used
 * for annotations. It reports whether there are cyclic dependencies
 * present in the graph.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class PrecedenceGraph<T> extends DirectedGraph<T> {

    var annotations = new NodeAnnotations<T>()

    /**
     * Indicates whether or not the graph has been analyzed for cycles.
     * If this variable is false, Tarjan's algorithm need to be ran to find out
     * whether or not this graph has cycles.
     */
    var cycleAnalysisDone = false

    /**
     * Indicates whether or not the graph has been sorted.
     * If this variable is false, a new DFS has to be done to compute a
     * topological sort.
     */
    var isSorted = false
    
    var index = 0
    
    var List<T> sortedNodes = emptyList
    
    var Stack<T> stack = new Stack()
    
    var List<Set<T>> cycles = emptyList

    override graphChanged() {
        this.cycleAnalysisDone = false
        this.isSorted = false
    }

    /**
     * Construct a new dependency graph.
     */
    new () {
        
    }
    
    private def void sortNodes() {
        
        if (!this.isSorted) {
            // Cleanup.
            this.sortedNodes = newLinkedList
            this.nodes.forEach [
                this.annotations.get(it).hasTempMark = false;
                this.annotations.get(it).hasPermMark = false
            ]

            // Start sorting.
            for (node : this.nodes) {
                if (!this.annotations.get(node).hasPermMark) {
                    // Unmarked node.
                    this.visit(node)
                }
            }
            this.isSorted = true
        }
    }
    
    
    def void visit(T node) {
        val annotation = this.annotations.get(node)
        if (annotation.hasPermMark) {
            return
        }
        if (annotation.hasTempMark) {
            // Not a DAG.
            throw new Error("Cycle found.")
        }
        annotation.hasTempMark = true
        for (dep : this.getEffects(node)) {
            visit(dep)
        }
        annotation.hasTempMark = false
        annotation.hasPermMark = true
        this.sortedNodes.add(node)
    }
    
    /**
     * Run Tarjan's algorithm for finding strongly connected components.
     * After invoking this method, the detected cycles with be listed 
     * in the class variable `cycles`.
     */
    def void detectCycles() {
        if (!this.cycleAnalysisDone) {
            this.index = 0
            this.stack = new Stack()
            this.cycles = new LinkedList();
            this.nodes.forEach[this.annotations.get(it).index = -1]
            for (node : this.nodes) {
                if (this.annotations.get(node).index == -1) {
                    this.strongConnect(node)
                }
            }    
        }
    }
    
    def hasCycles() {
        this.detectCycles
        if (this.cycles.size > 0) {
            return true
        }
        return false
    }
    
    def getCycles() {
        this.detectCycles
        return this.cycles
    }
    
    /**
     * Traverse the graph to visit unvisited dependencies and determine
     * whether they are part of a cycle. 
     */
    def void strongConnect(T node) {
        val annotation = this.annotations.get(node)
        annotation.index = this.index
        annotation.lowLink = this.index
        annotation.onStack = true
        this.index++
        this.stack.push(node)
        for (dep : this.getOrigins(node)) {
            val depAnnotation = this.annotations.get(dep)
            if (depAnnotation.onStack) {
                annotation.lowLink = Math.min(annotation.lowLink, depAnnotation.index)
                if (node.equals(dep)) {
                    annotation.selfLoop = true
                }
            } else if (depAnnotation.index == -1) {
                strongConnect(dep)    
                annotation.lowLink = Math.min(annotation.lowLink, depAnnotation.lowLink)
            } 
        }
        
        if (annotation.lowLink == annotation.index) {
            var scc = new HashSet()
            var T dep = null
            do {
                dep = this.stack.pop()
                this.annotations.get(dep).onStack = false
                scc.add(dep)
            } while(!node.equals(dep))
            // Only report self loops or cycles with two or more nodes.
            if (scc.size > 1 || annotation.selfLoop)
                this.cycles.add(scc)
        }   
    }
    
    def nodesOrderedAscending() {
        this.sortNodes()
        this.sortedNodes
    }
    
    def nodesOrderedDescending() {
        this.sortNodes()
        this.sortedNodes.reverse
    }
    
}

class NodeAnnotations<T> {
    var annotations = new HashMap<T, NodeAnnotation>
    
    def get(T node) {
        var annotation = this.annotations.get(node)
        if (annotation === null) {
            annotation = new NodeAnnotation()
            this.annotations.put(node, annotation)
        }
        return annotation
    }
    
    def put(T node, NodeAnnotation annotation) {
        this.annotations.put(node, annotation)
    }
    
}

/**
 * Node to be used in
 * {@link #AnnotatedDependencyGraph AnnotatedDependencyGraph}.
 * 
 * In particular, this is a helper class for its implementation
 * of Tarjan's algorithm for finding strongly connected components.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class NodeAnnotation {
    
    /**
     * Sequence number that is assigned when this node is discovered.
     * A node with a lower index was discovered later than this one;
     * a node with a higher index was discovered later than this one.
     */
    public var index = -1;
    
    /**
     * Temporary mark do be used in topological sort algorithm.
     */
    public var hasTempMark = false;
    
    /**
     * Temporary mark do be used in topological sort algorithm.
     */
    public var hasPermMark = false;
    
    /**
     * The smallest index of any node known to be reachable from this node.
     */
    public var lowLink = -1;
    
    /**
     * Whether or not this node is currently on the stack that
     * keeps track of visited nodes that potentially form a cycle.
     */
    public var onStack = false;
    
    /**
     * Whether or not this node has a dependency on itself.
     */
    public var selfLoop = false
    
}

