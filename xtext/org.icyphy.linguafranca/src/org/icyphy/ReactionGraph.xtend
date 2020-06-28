/* Dependency graph of a reactor program. */

/*************
Copyright (c) 2020, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

package org.icyphy

import org.eclipse.emf.ecore.EObject
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Model
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.VarRef
import java.util.HashSet
import java.util.Set

import static extension org.icyphy.ASTUtils.*

/**
 * A graph with vertices that are ports or reactions and edges that denote
 * dependencies between them.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
class ReactionGraph extends AnnotatedDependencyGraph<InstanceBinding> {

    /**
     * Construct a reaction graph based on the given AST and run Tarjan's
     * algorithm to detect cycles.
     * 
     * @param model The AST.
     */
    new (Model model) {
        // Find the main reactors and traverse the AST depth-first.
        collectNodesFrom(model.eAllContents.filter(Reactor).findFirst[it.main],
            new BreadCrumbTrail("", null, ""), new HashSet())
        this.detectCycles()
    }

    /**
     * Traverse the subtree that is a reactor definition and create graph nodes
     * for all of its components in relation to a particular instantiation that
     * is kept track of using a bread crumb trail.
     * 
     * @param reactor A reactor class definition.
     * @param instantiation The instantiation to bind the graph nodes to.
     */
    def void collectNodesFrom(Reactor reactor, BreadCrumbTrail<Instantiation> path, Set<String> visited) {
                
        // Nothing to do.
        if (reactor === null) {
            return   
        }
        
        // Add edges implied by connections.
        if (reactor.allConnections !== null) {
            for (c : reactor.connections) {
                // Ignore connections with delays because delays break cycles.
                if (c.delay === null) {
                    this.addEdge(new AnnotatedNode(
                        new InstanceBinding(c.rightPort.variable,
                            path.append(c.rightPort.container, c.rightPort.container?.name))),
                        new AnnotatedNode(
                            new InstanceBinding(c.leftPort.variable,
                                path.append(c.leftPort.container, c.leftPort.container?.name))))
                }
            }
        }

        if (reactor.allReactions !== null) {
            var Reaction prev = null
            // Iterate over the reactions.
            for (r : reactor.reactions) {
                // Add edges implied by reactions.
                val reaction = new AnnotatedNode(
                    new InstanceBinding(r as EObject, path))
                for (trigger : r.triggers.filter(VarRef)) {
                    if (trigger.variable instanceof Port) {
                        this.addEdge(reaction,
                            new AnnotatedNode(
                                new InstanceBinding(trigger.variable,
                                    path.append(trigger.container, trigger.container?.name))))
                    }
                }
                for (source : r.sources.
                    filter[it.variable instanceof Port]) {
                    this.addEdge(reaction,
                        new AnnotatedNode(
                            new InstanceBinding(source.variable,
                                path.append(source.container, source.container?.name))))
                }
                for (effect : r.effects.filter[it.variable instanceof Port]) {
                    this.addEdge(
                        new AnnotatedNode(
                            new InstanceBinding(effect.variable,
                                path.append(effect.container, effect.container?.name))),
                        reaction)
                }

                // Add edges implied by ordering of reactions within the reactor.
                if (prev !== null) {
                    this.addEdge(
                        reaction,
                        new AnnotatedNode(
                            new InstanceBinding(prev, path))
                    )
                }
                prev = r
            }
        }
        
        if (reactor.allInstantiations !== null) {
            for (inst : reactor.instantiations) {
                val breadCrumb = path.append(inst, inst.name)
                val breadCrumbStr = breadCrumb.toString
                // Do not visited nodes twice.
                if (!visited.contains(breadCrumbStr)) {
                    visited.add(breadCrumbStr)
                    this.collectNodesFrom(inst.reactorClass, breadCrumb, visited)
                }
            }
        }
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
    public BreadCrumbTrail<Instantiation> path
    
    /**
     * An arbitrary AST node.
     */
    public EObject node
        
    /**
     * Create a new binding to associate an arbitrary AST node with a particular
     * reactor instantiation.
     * 
     * @param node An arbitrary AST node.
     * @param path Path from the root of the AST to the given node. 
     */
    new(EObject node, BreadCrumbTrail<Instantiation> path) {
        this.node = node
        this.path = path
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
            return ((this.path === null && obj.path === null) ||
                (this.path !== null && this.path.equals(obj.path)) &&
                    this.node === obj.node)
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
        val prime = 31;
        var result = 1;
        result = prime * result +
            ((this.node === null) ? 0 : this.node.hashCode());
        result = prime * result +
            ((this.path === null) ? 0 : this.path.hashCode());
        return result;
    }
    
    /**
     * Return the instantiation that the node was bound to.
     */
    def getInstantiation() {
        return this.path.crumb
    }
    
    /**
     * Return a string representation of this binding consisting of
     * a path identifier combined with the name of the node.
     */
    override toString() {
        var String name = ""
        var prefix = this.path.toString
        try {
            name = this.node?.getClass.getDeclaredMethod("getName").invoke(
                this.node) as String
        } catch (Exception e) {
            name = this.node?.toString
        }
        '''«IF prefix !== null && !prefix.isEmpty»«prefix».«ENDIF»«name»'''
    }
}

/**
 * Class that serves to identify paths through the AST.
 * 
 * It assumed that the supplied identifiers uniquely identify each crumb when
 * combined with the identifiers that preceded it in a depth-first traversal
 * starting from the root of the tree. 
 */
class BreadCrumbTrail<T> {
    
    /**
     * An identifier built incrementally each time a new crumb is appended.
     */
    String trail
    
    /**
     * The last crumb that was appended. Together with the trail it identifies
     * a path through the AST.
     */
    T crumb
    
    /**
     * The next identifier to be appended to the trail.
     */
    String id
    
    /**
     * Create a new bread crumb trail.
     * 
     * @param trail A string prefix necessary to identify the crumb.
     * @param crumb The current crumb.
     * @param id Identifier associated with the current crumb. It will be
     * joined with trail string upon the next invocation of
     * {@link #BreadCrumbTrail.append append}.
     */    
    new(String trail, T crumb, String id) {
        this.trail = trail
        this.crumb = crumb
        this.id = id
    }
    
    /**
     * Append a new crumb to the trail.
     * 
     * If the crumb is null, just return the current object, otherwise return
     * a new bread crumb trail that has as a prefix the current trail.
     * @param The crumb to the added to the trail.
     * @param A string that identifies the crumb in combination with the trail
     * that has been constructed so far.
     */
    def BreadCrumbTrail<T> append(T crumb, String id) {
        if (crumb === null) {
            return this
        } else {
            return new BreadCrumbTrail(this.toString, crumb, id)
        }
    }
    
    /**
     * Report whether or not this object is equal to the given argument.
     * 
     * Two trails are equal if their string representations are equal.
     * @param obj The object to compare against.
     */
    override equals(Object obj) {
        if (obj instanceof BreadCrumbTrail) {
            return this.toString.equals(obj.toString)
        }
        return false
    }
    
    /**
     * Return the current trail which uniquely identifies the path from the
     * current crumb to the root of the tree. 
     */
    def getTrail() {
        return this.trail
    }
    
    /**
     * Return the current crumb.
     */
    def getCrumb() {
        return this.crumb
    }
    
    
    /**
     * Return a has code based on the string representation of this object.
     */
    override hashCode() {
        return this.toString.hashCode;
    }
    
    /**
     * Return the bread crumb trail including the trailing identifier
     * corresponding to the last crumb.
     */    
    override toString() {
        return '''«IF trail !== null && !trail.isEmpty»«this.trail».«ENDIF»«this.id»'''
    }
}
