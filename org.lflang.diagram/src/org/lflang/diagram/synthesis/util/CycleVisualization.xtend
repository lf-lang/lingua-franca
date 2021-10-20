package org.lflang.diagram.synthesis.util

import com.google.common.collect.HashMultimap
import com.google.inject.Inject
import de.cau.cs.kieler.klighd.kgraph.KEdge
import de.cau.cs.kieler.klighd.kgraph.KGraphElement
import de.cau.cs.kieler.klighd.kgraph.KNode
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared
import java.util.Map
import java.util.Set
import java.util.function.Consumer
import org.eclipse.elk.graph.properties.Property
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions
import org.lflang.generator.NamedInstance
import org.lflang.generator.ReactorInstance
import org.lflang.graph.TopologyGraph
import org.lflang.lf.Connection

import static extension org.lflang.diagram.synthesis.util.NamedInstanceUtil.*

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
	def boolean detectAndHighlightCycles(ReactorInstance rootReactorInstance, Map<ReactorInstance, KNode> allReactorNodes, Consumer<KGraphElement> highlighter) {
		val graph = new TopologyGraph(rootReactorInstance)
		
        if (!graph.cycles.empty && highlighter !== null) {
			// Highlight cycles
            for (cycle : graph.cycles) {
                // A cycle consists of reactions and ports, first find the involved reactor instances
                val cycleElementsByReactor = HashMultimap.create
                for (element : cycle) {
                    if (element instanceof ReactorInstance) {
                        cycleElementsByReactor.put(element, element)
                    } else {
                        cycleElementsByReactor.put(element.parent, element)
                    }
                }
                
                for (reactor : cycleElementsByReactor.keySet) {
                    val node = allReactorNodes.get(reactor)
                    if (node !== null) {
                        node.setProperty(DEPENDENCY_CYCLE, true)
                        highlighter.accept(node)

                        val reactorContentInCycle = cycleElementsByReactor.get(reactor)
                        
                        // Reactor edges
                        for (edge : node.outgoingEdges) {
                            if (edge.connectsCycleElements(cycle)) {
                                edge.setProperty(DEPENDENCY_CYCLE, true)
                                highlighter.accept(edge)
                            }
                        }

                        // Reactor ports
                        for (port : node.ports) {
                            if (reactorContentInCycle.contains(port.linkedInstance)) {
                                port.setProperty(DEPENDENCY_CYCLE, true)
                                highlighter.accept(port)
                            }
                        }

                        // Child Nodes
                        for (childNode : node.children) {
                            if (reactorContentInCycle.contains(childNode.linkedInstance) && !childNode.sourceIsReactor) {
                                childNode.setProperty(DEPENDENCY_CYCLE, true)
                                highlighter.accept(childNode)
    
                                for (edge : childNode.outgoingEdges) {
                                    if (edge.connectsCycleElements(cycle)) {
                                        edge.setProperty(DEPENDENCY_CYCLE, true)
                                        highlighter.accept(edge)
                                    }
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
	
	/**
	 * Checks whether an edge connects two elements that are part of the cycle.
	 * Assumes that the source node is always part of the cycle!
	 */
	private def boolean connectsCycleElements(KEdge edge, Set<NamedInstance<?>> cycle) {
        return (
                // if source is not a reactor, there is nothing to check
                !edge.source.sourceIsReactor
                ||
                // otherwise, the source port must be on the cycle
                cycle.contains(edge.sourcePort.linkedInstance)
            ) && (
                // leads to reactor port in cycle
                edge.target.sourceIsReactor && cycle.contains(edge.targetPort.linkedInstance)
                ||
                // leads to reaction in cycle
                !edge.target.sourceIsReactor && cycle.contains(edge.target.linkedInstance)
            ) && (
                // Special case only for connections
                !(edge.sourceElement() instanceof Connection)
                || (
                    // If the edge represents a connections between two ports in the cycle (checked before),
                    // then it is only included in the actual cycle, if it is neither delayed nor physical.
                    (edge.sourceElement() as Connection).delay === null
                    &&
                    !(edge.sourceElement() as Connection).physical
                )
            )
    }
}
