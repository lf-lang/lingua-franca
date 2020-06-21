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

package org.icyphy

import java.util.LinkedList
import java.util.List
import java.util.Stack
import java.util.Set
import java.util.HashSet

/** 
 * Elaboration of `DirectedGraph` that stores nodes in a wrapper used
 * for annotations 
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class AnnotatedDependencyGraph<T> extends DirectedGraph<AnnotatedNode<T>> {

    var index = 0
    var Stack<AnnotatedNode<T>> stack = new Stack()
    public var List<Set<AnnotatedNode<T>>> cycles = new LinkedList()

    /**
     * Construct a new dependency graph.
     */
    new() {
        
    }
    
    /**
     * Run Tarjan's algorithm for finding strongly connected components.
     * After invoking this method, the detected cycles with be listed 
     * in the class variable `cycles`.
     */
    def void detectCycles() {
        this.index = 0
        this.stack = new Stack()
        this.cycles = new LinkedList();
        for (node : this.nodes) {
            if (node.index == -1) {
                this.traverse(node)
            }
        }
    }
    
    /**
     * Traverse the graph to visit unvisited dependencies and determine
     * whether they are part of a cycle. 
     */
    def void traverse(AnnotatedNode<T> node) {
        node.index = this.index
        node.lowLink = this.index
        node.onStack = true
        this.index++
        this.stack.push(node)
        for (dep : this.getOrigins(node)) {
            
            if (dep.onStack) {
                node.lowLink = Math.min(node.lowLink, dep.index)
                if (node.equals(dep)) {
                    node.selfLoop = true
                }
            } else if (dep.index == -1) {
                traverse(dep)    
                node.lowLink = Math.min(node.lowLink, dep.lowLink)
            } 
        }
        
        if (node.lowLink == node.index) {
            var scc = new HashSet()
            var AnnotatedNode<T> dep = null
            do {
                dep = this.stack.pop()
                dep.onStack = false
                scc.add(dep)
            } while(!node.equals(dep))
            // Only report self loops or cycles with two or more nodes.
            if (scc.size > 1 || node.selfLoop)
                this.cycles.add(scc)
        }   
    }
}

