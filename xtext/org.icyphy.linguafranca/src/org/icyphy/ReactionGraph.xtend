package org.icyphy

import org.eclipse.emf.ecore.EObject
import org.icyphy.linguaFranca.Model
import org.icyphy.linguaFranca.Port
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.VarRef
import org.icyphy.linguaFranca.Instantiation

import static extension org.icyphy.ASTUtils.*
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

    def collectNodesFrom(Reactor reactor, Instantiation instantiation) {
        // Add edges implied by connections.
        if (reactor.connections !== null) {
            for (c : reactor.connections) {
                // Ignore connections with delays because delays break cycles.
                if (c.delay === null) {
                this.addEdge(new AnnotatedNode(
                    new InstanceBinding(c.rightPort.variable,
                        c.rightPort.getBinding(instantiation))),
                    new AnnotatedNode(new InstanceBinding(c.leftPort.variable,
                        c.leftPort.getBinding(instantiation))))
                }
            }
        }

        if (reactor.reactions !== null) {
            var Reaction prev = null
            // Iterate over the reactions.
            for (r : reactor.reactions) {
                // Add edges implied by reactions.
                val reaction = new AnnotatedNode(new InstanceBinding(r as EObject, instantiation))
                for (trigger : r.triggers.filter(VarRef)) {
                    if (trigger.variable instanceof Port) {
                        this.addEdge(reaction,
                            new AnnotatedNode(
                                new InstanceBinding(trigger.variable,
                                    trigger.getBinding(instantiation))))
                    }
                }
                for (source : r.sources.
                    filter[it.variable instanceof Port]) {
                    this.addEdge(reaction,
                        new AnnotatedNode(
                            new InstanceBinding(source.variable,
                                source.getBinding(instantiation))))
                }
                for (effect : r.effects.
                    filter[it.variable instanceof Port]) {
                    this.addEdge(
                        new AnnotatedNode(
                            new InstanceBinding(effect.variable,
                                effect.getBinding(instantiation))),
                        reaction)
                }

                // Add edges implied by ordering of reactions within a reactor.
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
     * 
     */
    public Instantiation instantiation
    
    /**
     * 
     */
    public EObject node
    
    /**
     * 
     */
    new(EObject node, Instantiation instantiation) {
        this.node = node
        this.instantiation = instantiation
        
    }
    
    /**
     * Return whether or not the given object is equal to this one.
     * 
     * If the given is an instance of the same class and its members
     * are equal, return true.
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
    
    override hashCode() {
        val prime = 31;
        var result = 1;
        result = prime * result + ((this.node === null) ? 0 : this.node.hashCode());
        result = prime * result + ((this.instantiation === null) ? 0 : this.instantiation.hashCode());
        return result;
    }
    
    
    
}
