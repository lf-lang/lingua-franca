/*************
* Copyright (c) 2020, Kiel University.
*
* Redistribution and use in source and binary forms, with or without modification,
* are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
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
		
        if (graph.hasCycles() && highlighter !== null) {
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
