package org.icyphy

import org.eclipse.emf.ecore.EObject
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
class ReactionGraph extends AnnotatedDependencyGraph<EObject> {

    /**
     * Construct a reaction graph based on the given AST and run Tarjan's
     * algorithm to detect cycles.
     */
    new (Model model) {
        for (reactor : model.eAllContents.toIterable.filter(Reactor)) {
            
            // Add edges implied by connections.
            if (reactor.connections !== null) {
                for (c : reactor.connections) {
                    // Ignore connections with delays because delays break cycles.
                    if (c.delay === null) {
                        this.addEdge(new AnnotatedNode(c.rightPort.variable),
                            new AnnotatedNode(c.leftPort.variable))
                    }
                }    
            }
            
            if (reactor.reactions !== null) {
                var Reaction prev = null
                // Iterate over the reactions.
                for (r : reactor.reactions) {
                    
                    // Add edges implied by reactions.
                    val reaction = new AnnotatedNode(r as EObject)
                    for (trigger : r.triggers.filter(VarRef)) {
                        if (trigger.variable instanceof Port) {
                            this.addEdge(reaction,
                            new AnnotatedNode(trigger.variable))
                        }
                    }
                    for (source : r.sources.filter[it | it.variable instanceof Port]) {
                        this.addEdge(reaction,
                        new AnnotatedNode(source.variable))
                    }
                    for (effect : r.effects.filter[it | it.variable instanceof Port]) {
                        this.addEdge(new AnnotatedNode(effect.variable),
                            reaction)
                    }
                    
                    // Add edges implied by ordering of reactions within a reactor.
                    if (prev !== null) {
                        this.addEdge(
                            reaction,
                            new AnnotatedNode(prev)
                        )
                    }
                    prev = r
                }    
            }
        }
        this.detectCycles()
    }
}