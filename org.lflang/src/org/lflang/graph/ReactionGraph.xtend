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

import org.eclipse.emf.ecore.EObject
import org.lflang.lf.Instantiation
import org.lflang.lf.Model
import org.lflang.lf.Port
import org.lflang.lf.Reaction
import org.lflang.lf.Reactor
import org.lflang.lf.VarRef

import static extension org.lflang.ASTUtils.*
import java.util.LinkedList
import java.util.List

/**
 * A graph with vertices that are ports or reactions and edges that denote
 * dependencies between them.
 * 
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class ReactionGraph extends PrecedenceGraph<InstanceBinding> {

    /**
     * Construct a reaction graph based on the given AST and run Tarjan's
     * algorithm to detect cyclic dependencies between reactions. It is
     * assumed that no instantiation cycles are present in the program.
     * Checks for instantiation cycles thus must be carried out prior to  
     * constructing the reaction graph.
     * @param model The AST in which to find the main reactor to construct the graph for.
     */
    new(Model model) {
        // Collect nodes from the main reactor and all instances recursively contained in it.
        this(model.reactors.findFirst[main || federated])
    }

    /**
     * Construct a reaction graph based on the given reactor and run Tarjan's
     * algorithm to detect cyclic dependencies between reactions. It is
     * assumed that no instantiation cycles are present in the program.
     * Checks for instantiation cycles thus must be carried out prior to  
     * constructing the reaction graph.
     * @param model The reactor to construct the graph for.
     */
    new(Reactor reactor) {
        collectNodesFrom(new InstanceBinding(reactor))
        this.detectCycles()
    }


    /**
     * Traverse the subtree that is a reactor definition that is bound to a particular series of instantiations.
     * 
     * @param node A bound reactor class instantiation.
     */
    def void collectNodesFrom(InstanceBinding node) {

        if (node === null) {
            // Nothing to do.
            return
        }
        
        val last = node.path.last
        var Reactor reactor
        if (last instanceof Reactor) {
            reactor = last as Reactor
        } else if (last instanceof Instantiation) {
            reactor = (last as Instantiation).reactorClass.toDefinition
        } else {
            // Nothing to do.
            return
        }
        
        // Add edges implied by connections.
        if (reactor.allConnections !== null) {
            for (c : reactor.allConnections) {
                // Ignore connections with delays because delays break cycles.
                if (c.delay === null) {
                    // For parallel connections, whether a dependency actually exists
                    // depends on the widths.
                    var rightPort = c.rightPorts.get(0)
                    var remainingRightPorts = rightPort.inferPortWidth(c, node.instantiations.toList)
                    var rightPortCount = 0
                    for (leftPort : c.leftPorts) {
                        var remainingLeftPorts = leftPort.inferPortWidth(c, node.instantiations.toList)
                        addEdge(leftPort, rightPort, node)
                        // How many ports were connected by this edge?
                        var connected = (remainingLeftPorts <
                                remainingRightPorts) ? remainingLeftPorts : remainingRightPorts
                        remainingLeftPorts -= connected
                        remainingRightPorts -= connected
                        // If this left port needs more right ports to fill out its
                        // connections, establish those edges as well.
                        while (remainingLeftPorts > 0) {
                            rightPortCount++
                            rightPort = c.rightPorts.get(rightPortCount)
                            remainingRightPorts = rightPort.inferPortWidth(c, node.instantiations.toList)
                            addEdge(leftPort, rightPort, node)
                            // How many ports were connected by this edge?
                            connected = (remainingLeftPorts <
                                remainingRightPorts) ? remainingLeftPorts : remainingRightPorts
                            remainingLeftPorts -= connected
                            remainingRightPorts -= connected
                        }
                        if (remainingRightPorts === 0 && rightPortCount < c.rightPorts.length - 1) {
                            rightPortCount++
                            rightPort = c.rightPorts.get(rightPortCount)
                            remainingRightPorts = rightPort.inferPortWidth(c, node.instantiations.toList)
                        }
                        
                    }
                }
            }
        }

        if (reactor.allReactions !== null) {
            var Reaction prev = null
            // Iterate over the reactions.
            for (r : reactor.reactions) {
                // Add edges implied by reactions.
                val reaction = new InstanceBinding(node, r)
                for (trigger : r.triggers.filter(VarRef)) {
                    if (trigger.variable instanceof Port) {
                        this.addEdge(reaction, InstanceBinding.bind(node, trigger))
                    }
                }
                for (source : r.sources.filter[it.variable instanceof Port]) {
                     this.addEdge(reaction, InstanceBinding.bind(node, source))
                }
                for (effect : r.effects.filter[it.variable instanceof Port]) {
                    this.addEdge(reaction, InstanceBinding.bind(node, effect))
                }

                // Add edges implied by ordering of reactions within the reactor.
                if (prev !== null) {
                    this.addEdge(
                        reaction,
                        new InstanceBinding(node, prev)
                    )
                }
                prev = r
            }
        }

        if (reactor.allInstantiations !== null) {
            for (inst : reactor.instantiations) {
                this.collectNodesFrom(new InstanceBinding(node, inst))
            }
        }
    }

    /**
     * Add an edge between the given left
     * @param leftPort The left port.
     * @param rightPort The right port.
     * @param path The breadcrumb trail.
     */
    def addEdge(VarRef leftPort, VarRef rightPort, InstanceBinding connectionContainer) {
        this.addEdge(InstanceBinding.bind(connectionContainer, rightPort),
            InstanceBinding.bind(connectionContainer, leftPort))
        
    }
}

/**
 * Binds an AST node to the instantiation of the reactor that it is a part of.
 */
class InstanceBinding {

    /**
     * Object to distinguish between different paths from the root of the tree
     * to the current node, and access the instantiation to bind to.
     */
    public List<EObject> path = new LinkedList();
    
    val Object node = null
    
    /**
     * Create an instance binding based on the given parent binding and a list of children to append to it.
     * @param parent Parent binding that represents a path via which to reach the given children.
     * @param children AST nodes reachable from the path in the given parent binding.
     */
    new (InstanceBinding parent, EObject ...children) {
        parent.path.forEach[path.add(it)]
        children.forEach[path.add(it)]
    }

    /**
     * Create an empty instance binding, meaning there are no parent nodes in the instantiation tree of this binding. 
     */
    new(Reactor main) {
        path.add(main)
    }

    /**
     * Return whether or not the given object is equal to this one.
     * 
     * If the given object is an instance of the same class and it creates
     * an identical binding, then return true.
     * @param obj The object to compare against.
     */
    override equals(Object obj) {
        if (obj instanceof InstanceBinding) {
            return ((this.path === null && obj.path === null) || (this.path !== null && this.path.equals(obj.path)))
        }
        return false
    }

    /**
     * Return the hash code of this object.
     * 
     * Combine the hash codes of the AST nodes that this object establishes
     * a binding between.
     */
    override hashCode() {
        return path.hashCode
    }

    /**
     * Return the list of instantiations that the node was bound to.
     */
    def getInstantiations() {
        val instantiations = path.filter(Instantiation)
        if (this.node instanceof Instantiation) {
            return instantiations.take(instantiations.size-1)
        } else {
            return instantiations
        }
    }

    def getParent() {
        return this.instantiations.last
    }
    
    def getNode() {
        return path.last
    }

    /**
     * Return a string representation of this binding consisting of
     * a path identifier combined with the name of the node.
     */
    override toString() {
        this.path.fold(new StringBuilder(), [sb, node | 
            var String name = ""
            try {
                name = this.node?.getClass.getDeclaredMethod("getName").invoke(this.node) as String
            } catch (Exception e) {
                name = this.node?.toString
            }
            sb.append("." + name)
        ]).toString
    }
    
    static def bind(InstanceBinding node, VarRef ref) {
        if (ref.container !== node.path.last) {
            return new InstanceBinding(node, ref.container, ref.variable)
        } else {
            return new InstanceBinding(node, ref.variable)
        }
    }
    
}
