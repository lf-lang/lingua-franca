package org.lflang.diagram.synthesis.util

import com.google.common.collect.HashMultimap
import com.google.inject.Inject
import de.cau.cs.kieler.klighd.kgraph.KGraphElement
import de.cau.cs.kieler.klighd.kgraph.KNode
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared
import java.util.Map
import java.util.function.Consumer
import org.eclipse.elk.graph.properties.Property
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions
import org.lflang.diagram.synthesis.LinguaFrancaSynthesis
import org.lflang.generator.ReactorInstance
import org.lflang.graph.TopologyGraph
import org.lflang.lf.Connection

/**
 * Dependency cycle detection for Lingua Franca diagrams.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
@ViewSynthesisShared
class CycleVisualization extends AbstractSynthesisExtensions {
	
	// Properties for marking diagram elements
	public static val DEPENDENCY_CYCLE = new Property<Boolean>("org.lflang.diagram.synthesis.dependency.cycle", false)
	
	@Inject
	extension UtilityExtensions
	
	/**
	 * Performs cycle detection based on the diagram's graph structure and applies given highlighting to the included elements
	 */
	def boolean detectAndHighlightCycles(ReactorInstance reactorInstance, Map<ReactorInstance, KNode> allReactorNodes, Consumer<KGraphElement> highlighter) {
		val graph = new TopologyGraph(reactorInstance)
		
        if (!graph.cycles.empty && highlighter !== null) {
			// Highlight cycles
            for (cycle : graph.cycles) {
                // FIXME: Why is cycle being copied into a multimap?
            	val allAffectedElements = HashMultimap.create
            	for (elem : cycle) {
            		allAffectedElements.put(elem, elem.definition)
            	}
            	
            	for (reactorCrumb : allAffectedElements.keySet) {
            	    // reactorCrumb is either a ReactionInstance or a PortInstance.
            	    // In either case, the reactor node is the parent.
            		val affectedElements = allAffectedElements.get(reactorCrumb)
            		val reactorNode = allReactorNodes.get(reactorCrumb.parent)
            		// This should not be null, but we check anyway.
            		if (reactorNode !== null) {
                        reactorNode.setProperty(DEPENDENCY_CYCLE, true)
                        highlighter.accept(reactorNode)

                        // Reactor edges
                        for (cycleEgde : reactorNode.outgoingEdges.filter [
                            affectedElements.contains(sourcePort.sourceElement()) && (
            				(
            					!target.sourceIsReactor() && allAffectedElements.values.contains(target.sourceElement())
            				) || (
            					target.sourceIsReactor() &&
                                allAffectedElements.keySet.contains(
                                    target.getProperty(LinguaFrancaSynthesis.REACTOR_INSTANCE)) &&
                                allAffectedElements.get(target.getProperty(LinguaFrancaSynthesis.REACTOR_INSTANCE)).
                                    contains(targetPort.sourceElement())
            				)
            			)
                        ]) {
                            // FIXME: Still hard-coded semantics
                            if (!(cycleEgde.sourceElement() instanceof Connection) ||
                                (cycleEgde.sourceElement() as Connection).delay === null) {
                                cycleEgde.setProperty(DEPENDENCY_CYCLE, true)
                                highlighter.accept(cycleEgde)
                            }
                        }

                        // Reactor ports
                        for (cyclePort : reactorNode.ports.filter[affectedElements.contains(it.sourceElement())]) {
                            cyclePort.setProperty(DEPENDENCY_CYCLE, true)
                            highlighter.accept(cyclePort)
                        }

                        // Child Nodes
                        for (childNode : reactorNode.children.filter [
                            affectedElements.contains(it.sourceElement()) && !sourceIsReactor
                        ]) {
                            childNode.setProperty(DEPENDENCY_CYCLE, true)
                            highlighter.accept(childNode)

                            for (cycleEgde : childNode.outgoingEdges.filter [
                                (
            					!target.sourceIsReactor() && affectedElements.contains(target.sourceElement())
            				) || (
            					target.sourceIsReactor() &&
                                    allAffectedElements.keySet.contains(
                                        target.getProperty(LinguaFrancaSynthesis.REACTOR_INSTANCE)) &&
                                    allAffectedElements.get(target.getProperty(LinguaFrancaSynthesis.REACTOR_INSTANCE)).
                                        contains(targetPort.sourceElement())
            				)
                            ]) {
                                // FIXME: Still hard-coded semantics
                                if (!(cycleEgde.sourceElement() instanceof Connection) ||
                                    (cycleEgde.sourceElement() as Connection).delay === null) {
                                    cycleEgde.setProperty(DEPENDENCY_CYCLE, true)
                                    highlighter.accept(cycleEgde)
                                }
                            }
                        }
                    }
            	}
            }
            return true
         }
         
         return false
	}
}
