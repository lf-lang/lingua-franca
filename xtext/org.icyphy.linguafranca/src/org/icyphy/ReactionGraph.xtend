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
        // Treat the main reactor separately because it doesn't have an instantiation.
        collectNodesFrom(model.eAllContents.filter(Reactor).findFirst[it.main], new BreadCrumbTrail("", null, ""))
        // Collect nodes from all instantiations
        this.detectCycles()
    }

    /**
     * Traverse the subtree that is a reactor definition and create graph nodes
     * for all of its components in relation to a particular instantiation.
     * 
     * @param reactor A reactor class definition.
     * @param instantiation The instantiation to bind the graph nodes to.
     */
    def void collectNodesFrom(Reactor reactor, BreadCrumbTrail<Instantiation> path) {
                
        // Nothing to do.
        if (reactor === null) {
            return   
        }
        
        // Add edges implied by connections.
        if (reactor.connections !== null) {
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

        if (reactor.reactions !== null) {
            var Reaction prev = null
            // Iterate over the reactions.
            for (r : reactor.reactions) {
                // Add edges implied by reactions.
                val reaction = new AnnotatedNode(
                    new InstanceBinding(r as EObject, path))
                    // If the container is empty, then simply reuse the _same path_ (no need to create a new crumb!)
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
        
        if (reactor.instantiations !== null) {
            for (inst : reactor.instantiations) {
                this.collectNodesFrom(inst.reactorClass, path.append(inst, inst.name))    
            }
        }
    }
}

/**
 * Binds an AST node to the instantiation of the reactor that it is a part of.
 */
class InstanceBinding {
    
    /**
     * An AST node that denotes reactor instantiation.
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
     * @param inst The reactor instantiation to bind the node to. 
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
                (this.path !== null && this.path.equals(obj.path)) && this.node === obj.node)
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
    
    def getInstantiation() {
        return this.path.crumb
    }
    
    override toString() {
        var String name = ""
        var prefix = this.path.toString
        try {
            name = this.node?.getClass.getDeclaredMethod("getName").invoke(this.node) as String
        } catch (Exception e) {
            name = this.node?.toString
        }
        '''«IF prefix !== null && !prefix.isEmpty»«prefix».«ENDIF»«name»'''
    }
}

class BreadCrumbTrail<T> {
    
    String trail
    T crumb
    String id
        
    new(String trail, T crumb, String id) {
        this.trail = trail
        this.crumb = crumb
        this.id = id
    }
    
    def BreadCrumbTrail<T> append(T crumb, String id) {
        if (crumb === null) {
            return this
        } else {
            return new BreadCrumbTrail(this.toString, crumb, id)
        }
    }
    
    override equals(Object obj) {
        if (obj instanceof BreadCrumbTrail) {
            return this.toString.equals(obj.toString)
        }
        return false
    }
    
    override hashCode() {
        return this.toString.hashCode;
    }
    
    def getTrail() {
        return this.trail
    }
    
    def getCrumb() {
        return this.crumb
    }
    
    override toString() {
        return '''«IF trail !== null && !trail.isEmpty»«this.trail».«ENDIF»«this.id»'''
    }
}
