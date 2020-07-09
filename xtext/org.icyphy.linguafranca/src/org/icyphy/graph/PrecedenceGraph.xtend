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

/** 
 * Elaboration of `DirectedGraph` that stores nodes in a wrapper used
 * for annotations. It reports whether there are cyclic dependencies
 * present in the graph.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class PrecedenceGraph<T> implements DirectedGraph<T> {

    var graph = new SimpleDirectedGraph<AnnotatedNode<T>>

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
    
    var Stack<AnnotatedNode<T>> stack = new Stack()
    
    var List<Set<T>> cycles = emptyList

    private def graphChanged() {
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
            this.graph.nodes.forEach [
                it.hasTempMark = false;
                it.hasPermMark = false
            ]

            // Start sorting.
            for (node : this.graph.nodes) {
                if (!node.hasPermMark) {
                    // Unmarked node.
                    this.visit(node)
                }
            }
            this.isSorted = true
        }
    }
    
    
    def void visit(AnnotatedNode<T> node) {
        if (node.hasPermMark) {
            return
        }
        if (node.hasTempMark) {
            // Not a DAG.
            throw new Error("Cycle found.")
        }
        node.hasTempMark = true
        for (dep : this.graph.getEffects(node)) {
            visit(dep)
        }
        node.hasTempMark = false
        node.hasPermMark = true
        this.sortedNodes.add(node.contents)
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
            this.graph.nodes.forEach[it.index = -1]
            for (node : this.graph.nodes) {
                if (node.index == -1) {
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
    def void strongConnect(AnnotatedNode<T> node) {
        node.index = this.index
        node.lowLink = this.index
        node.onStack = true
        this.index++
        this.stack.push(node)
        for (dep : this.graph.getOrigins(node)) {
            
            if (dep.onStack) {
                node.lowLink = Math.min(node.lowLink, dep.index)
                if (node.equals(dep)) {
                    node.selfLoop = true
                }
            } else if (dep.index == -1) {
                strongConnect(dep)    
                node.lowLink = Math.min(node.lowLink, dep.lowLink)
            } 
        }
        
        if (node.lowLink == node.index) {
            var scc = new HashSet()
            var AnnotatedNode<T> dep = null
            do {
                dep = this.stack.pop()
                dep.onStack = false
                scc.add(dep.contents)
            } while(!node.equals(dep))
            // Only report self loops or cycles with two or more nodes.
            if (scc.size > 1 || node.selfLoop)
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
    
    override getOrigins(T node) {
        this.graph.getOrigins(new AnnotatedNode(node)).map[it.contents]
    }
    
    override getEffects(T node) {
        this.graph.getEffects(new AnnotatedNode(node)).map[it.contents]
    }
    
    override rootNodes() {
        this.graph.rootNodes.map[it.contents]
    }
    
    override leafNodes() {
        this.graph.leafNodes.map[it.contents]
    }
    
    override hasNode(T node) {
        this.graph.hasNode(new AnnotatedNode(node))
    }
    
    override addNode(T node) {
        this.graph.addNode(new AnnotatedNode(node))
    }
    
    override removeNode(T node) {
        graphChanged()
        this.graph.removeNode(new AnnotatedNode(node))
    }
    
    override addEdge(T to, T from) {
        graphChanged()
        this.graph.addEdge(new AnnotatedNode(to), new AnnotatedNode(from))
    }
    
    override addEdges(T to, List<T> from) {
        graphChanged()
        this.graph.addEdges(new AnnotatedNode(to), from.map[new AnnotatedNode(it)])
    }
    
    override removeEdge(T to, T from) {
        graphChanged()
        this.graph.removeEdge(new AnnotatedNode(to), new AnnotatedNode(from))
    }
    
    override nodeCount() {
        this.graph.nodeCount
    }
    
    override edgeCount() {
        this.graph.edgeCount
    }
    
    override nodes() {
        this.graph.nodes.map[it.contents]
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
class AnnotatedNode<T> {
    
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
     * The contents of this node.
     */
    public var T contents = null;
    
    /**
     * Whether or not this node has a dependency on itself.
     */
    public var selfLoop = false
    
    /**
     * Create a new annotated node providing its contents.
     */
    new(T value) {
        this.contents = value;
    }
    
    /**
     * Two annotated nodes are considered equal if their contents is equal.
     */
    override equals(Object obj) {
        if (obj instanceof AnnotatedNode){
            return contents.equals(obj.contents);    
        }
        return false
    }
    
    /**
     * Return the hash code of the node's contents.
     */
    override hashCode() {
        return contents.hashCode
    }

    /**
     * Return the string representation of the node's contents.
     */    
    override toString() {
        return contents.toString
    }
    
    static def <T> List<T> unpack(List<AnnotatedNode<T>> list) {
        return list.map[it.contents]
    }
}

