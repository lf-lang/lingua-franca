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
import org.icyphy.linguaFranca.Model
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.VarRef
import org.icyphy.linguaFranca.Instantiation

import org.icyphy.linguaFranca.Reactor

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
        collectNodesFrom(model.eAllContents.filter(Reactor).findFirst[it.main], null)
        // Collect nodes from all instantiations
        for (instantiation : model.eAllContents.toIterable.filter(Instantiation)) {
            collectNodesFrom(instantiation.reactorClass, instantiation)
        }
        this.detectCycles()
    }

    /**
     * Traverse the subtree that is a reactor definition and create graph nodes
     * for all of its components in relation to a particular instantiation.
     * 
     * @param reactor A reactor class definition.
     * @param instantiation The instantiation to bind the graph nodes to.
     */
    def collectNodesFrom(Reactor reactor, Instantiation instantiation) {
        
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
                        c.rightPort, instantiation)),
                    new AnnotatedNode(new InstanceBinding(c.leftPort.variable,
                        c.leftPort, instantiation)))
                }
            }
        }

        if (reactor.reactions !== null) {
            var Reaction prev = null
            // Iterate over the reactions.
            for (r : reactor.reactions) {
                // Add edges implied by reactions.
                val reaction = new AnnotatedNode(
                    new InstanceBinding(r as EObject, instantiation))
                for (trigger : r.triggers.filter(VarRef)) {
                    if (trigger.variable instanceof Port) {
                        this.addEdge(reaction,
                            new AnnotatedNode(
                                new InstanceBinding(trigger.variable,
                                    trigger, instantiation)))
                    }
                }
                for (source : r.sources.
                    filter[it.variable instanceof Port]) {
                    this.addEdge(reaction,
                        new AnnotatedNode(
                            new InstanceBinding(source.variable,
                                source, instantiation)))
                }
                for (effect : r.effects.
                    filter[it.variable instanceof Port]) {
                    this.addEdge(
                        new AnnotatedNode(
                            new InstanceBinding(effect.variable,
                                effect, instantiation)),
                        reaction)
                }

                // Add edges implied by ordering of reactions within the reactor.
                if (prev !== null) {
                    this.addEdge(
                        reaction,
                        new AnnotatedNode(
                            new InstanceBinding(prev, instantiation))
                    )
                }
                prev = r
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
    public Instantiation instantiation
    
    /**
     * An arbitrary AST node.
     */
    public EObject node
    
    /**
     * Create a new binding to associate an arbitrary AST node with a particular
     * reactor instantiation.
     * 
     * @param node An arbitrary AST node.
     * @param inst An AST node that denotes reactor instantiation. 
     */
    new(EObject node, Instantiation inst) {
        this.node = node
        this.instantiation = instantiation
    }
    
    /**
     * Create a new binding to associate an arbitrary AST node with a particular
     * reactor instantiation. 
     * 
     * If the supplied reference has a container, then use that as the
     * instantiation to bind to. Otherwise, use the instantiation that is passed
     * in as the third argument.
     * 
     * @param node An arbitrary AST node.
     * @param ref A reference to variable (e.g, a port).
     * @param inst An AST node that denotes reactor instantiation. 
     */
    new(EObject node, VarRef ref, Instantiation inst) {
        this.node = node
        this.instantiation = (ref.container === null) ? inst : ref.container
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
            return (this.instantiation === obj.instantiation && 
                    this.node === obj.node
            );
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
            ((this.instantiation === null) ? 0 : this.instantiation.hashCode());
        return result;
    }
}
