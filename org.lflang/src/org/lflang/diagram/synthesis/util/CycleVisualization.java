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
package org.lflang.diagram.synthesis.util;

import com.google.common.collect.HashMultimap;
import com.google.inject.Inject;
import de.cau.cs.kieler.klighd.kgraph.KEdge;
import de.cau.cs.kieler.klighd.kgraph.KGraphElement;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.kgraph.KPort;
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import org.eclipse.elk.graph.properties.Property;
import org.eclipse.xtext.xbase.lib.Extension;
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions;
import org.lflang.generator.NamedInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Connection;

/**
 * Dependency cycle detection for Lingua Franca diagrams.
 * 
 * @author Alexander Schulz-Rosengarten
 */
@ViewSynthesisShared
public class CycleVisualization extends AbstractSynthesisExtensions {
    
    // Properties for marking diagram elements
    public static final Property<Boolean> DEPENDENCY_CYCLE = new Property<>("org.lflang.diagram.synthesis.dependency.cycle", false);

    @Inject @Extension private UtilityExtensions _utilityExtensions;
    
    /**
     * Performs cycle detection based on the diagram's graph structure and applies given highlighting to the included elements
     */
    public boolean detectAndHighlightCycles(ReactorInstance rootReactorInstance, 
            Map<ReactorInstance, KNode> allReactorNodes, 
            Consumer<KGraphElement> highlighter) {
        
        if (rootReactorInstance.hasCycles() && highlighter != null) {
            // Highlight cycles
            // A cycle consists of reactions and ports.
            HashMultimap<ReactorInstance, NamedInstance<?>> cycleElementsByReactor = HashMultimap.create();
            Set<NamedInstance<?>> cycles = rootReactorInstance.getCycles();
            for (NamedInstance<?> element : cycles) {
                // First find the involved reactor instances
                if (element instanceof ReactorInstance) {
                    cycleElementsByReactor.put((ReactorInstance) element, element);
                } else {
                    cycleElementsByReactor.put(element.getParent(), element);
                }
            }
                
            for (ReactorInstance reactor  : cycleElementsByReactor.keySet()) {
                KNode node = allReactorNodes.get(reactor);
                if (node != null) {
                    node.setProperty(DEPENDENCY_CYCLE, true);
                    highlighter.accept(node);

                    Set<NamedInstance<?>> reactorContentInCycle = cycleElementsByReactor.get(reactor);
                    
                    // Reactor edges
                    for (KEdge edge : node.getOutgoingEdges()) {
                        if (connectsCycleElements(edge, cycles)) {
                            edge.setProperty(DEPENDENCY_CYCLE, true);
                            highlighter.accept(edge);
                        }
                    }

                    // Reactor ports
                    for (KPort port : node.getPorts()) {
                        if (reactorContentInCycle.contains(NamedInstanceUtil.getLinkedInstance(port))) {
                            port.setProperty(DEPENDENCY_CYCLE, true);
                            highlighter.accept(port);
                        }
                    }

                    // Child Nodes
                    for (KNode childNode : node.getChildren()) {
                        if (reactorContentInCycle.contains(NamedInstanceUtil.getLinkedInstance(childNode)) && 
                                !_utilityExtensions.sourceIsReactor(childNode)) {
                            childNode.setProperty(DEPENDENCY_CYCLE, true);
                            highlighter.accept(childNode);

                            for (KEdge edge : childNode.getOutgoingEdges()) {
                                if (connectsCycleElements(edge, cycles)) {
                                    edge.setProperty(DEPENDENCY_CYCLE, true);
                                    highlighter.accept(edge);
                                }
                            }
                        }
                    }
                }
            }
            return true;
         }
         return false;
    }
    
    /**
     * Checks whether an edge connects two elements that are part of the cycle.
     * Assumes that the source node is always part of the cycle!
     */
    private boolean connectsCycleElements(KEdge edge, Set<NamedInstance<?>> cycle) {
        return (
                // if source is not a reactor, there is nothing to check
                !_utilityExtensions.sourceIsReactor(edge.getSource())
                ||
                // otherwise, the source port must be on the cycle
                cycle.contains(NamedInstanceUtil.getLinkedInstance(edge.getSourcePort()))
            ) && (
                // leads to reactor port in cycle
                _utilityExtensions.sourceIsReactor(edge.getTarget()) 
                && 
                cycle.contains(NamedInstanceUtil.getLinkedInstance(edge.getTargetPort()))
                ||
                // leads to reaction in cycle
                !_utilityExtensions.sourceIsReactor(edge.getTarget()) 
                && 
                cycle.contains(NamedInstanceUtil.getLinkedInstance(edge.getTarget()))
            ) && (
                // Special case only for connections
                !(_utilityExtensions.sourceElement(edge) instanceof Connection)
                || (
                    // If the edge represents a connections between two ports in the cycle (checked before),
                    // then it is only included in the actual cycle, if it is neither delayed nor physical.
                    ((Connection) _utilityExtensions.sourceElement(edge)).getDelay() == null
                    &&
                    !((Connection) _utilityExtensions.sourceElement(edge)).isPhysical()
                )
            );
    }
}
