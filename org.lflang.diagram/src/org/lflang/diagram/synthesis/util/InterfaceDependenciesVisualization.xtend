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

import com.google.common.collect.Sets
import com.google.inject.Inject
import de.cau.cs.kieler.klighd.SynthesisOption
import de.cau.cs.kieler.klighd.kgraph.KNode
import de.cau.cs.kieler.klighd.kgraph.KPort
import de.cau.cs.kieler.klighd.krendering.KContainerRendering
import de.cau.cs.kieler.klighd.krendering.KInvisibility
import de.cau.cs.kieler.klighd.krendering.KRendering
import de.cau.cs.kieler.klighd.krendering.KRenderingFactory
import de.cau.cs.kieler.klighd.krendering.LineStyle
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KEdgeExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions
import de.cau.cs.kieler.klighd.syntheses.DiagramSyntheses
import de.cau.cs.kieler.klighd.util.KlighdProperties
import java.util.List
import java.util.Random
import org.eclipse.elk.core.math.ElkMargin
import org.eclipse.elk.core.options.CoreOptions
import org.eclipse.elk.graph.properties.Property
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions
import org.lflang.diagram.synthesis.LinguaFrancaSynthesis
import org.lflang.diagram.synthesis.styles.LinguaFrancaShapeExtensions
import org.lflang.diagram.synthesis.styles.LinguaFrancaStyleExtensions

/**
 * Utility class to handle dependency edges for collapsed reactors in Lingua Franca diagrams.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
@ViewSynthesisShared
class InterfaceDependenciesVisualization extends AbstractSynthesisExtensions {
    
    // Related synthesis option
    public static val SynthesisOption SHOW_INTERFACE_DEPENDENCIES = SynthesisOption.createCheckOption("Port Dependencies in Collapsed Reactors", false).setCategory(LinguaFrancaSynthesis.APPEARANCE)
	
	// Properties for marking diagram elements
	public static val INTERFACE_DEPENDENCY = new Property<Boolean>("org.lflang.linguafranca.diagram.synthesis.dependency.interface", false)
	
    @Inject extension KEdgeExtensions
    @Inject extension KRenderingExtensions
    @Inject extension KContainerRenderingExtensions
    @Inject extension LinguaFrancaStyleExtensions
    @Inject extension UtilityExtensions
    
    /**
     * Updates the visibility of interface dependencies edges based on the expansion state.
     */
    static def updateInterfaceDependencyVisibility(KNode node, boolean expanded) {
        node.outgoingEdges.filter[getProperty(INTERFACE_DEPENDENCY)].map[data.filter(typeof(KRendering))].flatten.forEach[
            var inv = styles.filter(typeof(KInvisibility)).last
            if (inv === null) {
                inv = KRenderingFactory.eINSTANCE.createKInvisibility()
                styles += inv
            }
            inv.invisible = expanded
        ]
    }

    /**
     * Adds interface dependencies to the node if this option is active.
     * Visibility will be adjusted based on expansion state.
     */
    def addInterfaceDependencies(KNode node, boolean expanded) {
        if (SHOW_INTERFACE_DEPENDENCIES.booleanValue) {
            val deps = node.portDependencies
            if (!deps.empty) {
                for (pair : deps) {
                    createDependencyEdge(pair, expanded)
                }
                
                // Fix content (label) of collapsed rendering
                var contentContainer = node.data.filter(KContainerRendering).findFirst[getProperty(KlighdProperties.COLLAPSED_RENDERING)]
                if (contentContainer !== null) {
                    if (!contentContainer.getProperty(LinguaFrancaShapeExtensions.REACTOR_CONTENT_CONTAINER)) {
                        contentContainer = contentContainer.eAllContents.filter(KContainerRendering).findFirst[getProperty(LinguaFrancaShapeExtensions.REACTOR_CONTENT_CONTAINER)]
                    }
                    if (contentContainer !== null) {
                        val content = contentContainer.children.immutableCopy
                        // Put into two new containers such that they are not centered/maximized
                        contentContainer.addRectangle() => [
                            invisible = true
                            addRectangle() => [
                                invisible = true
                                setGridPlacement(1)
                                children += content
                                
                                setPointPlacementData(LEFT, 0, 0.5f, TOP, 0, 0, H_CENTRAL, V_TOP, 0, 0, 0, 0)
                            ]
                        ]
                        
                        // Adjust ports separate dependency edges from label/content
                        if (content.size > 0) {
                            node.getPortMarginsInitIfAbsent().add(new ElkMargin((content.size * 20) - 8, 0, 0, 0))
                        }
                    }
                }
            }
        }
    }
    
    /**
     * Find dependencies between ports.
     */
    private def List<Pair<KPort, KPort>>getPortDependencies(KNode node) {
        val inputPorts = node.ports.filter[getProperty(LinguaFrancaSynthesis.REACTOR_INPUT)].toSet
        val outputPorts = node.ports.filter[getProperty(LinguaFrancaSynthesis.REACTOR_OUTPUT)].toSet
        
        // FIXME Replace with real logic
        val rand = new Random()
        return Sets.cartesianProduct(inputPorts, outputPorts).filter[rand.nextBoolean].map[new Pair(get(0), get(1))].toList
    }
    
    /**
     * Create an edges for interface dependencies and adjust visibility based on the expansion state of the node.
     */
    private def createDependencyEdge(Pair<KPort, KPort> connection, boolean expanded) {
        return createEdge => [
            source = connection.key.node
            sourcePort = connection.key
            target = connection.value.node
            targetPort = connection.value
            
            setProperty(INTERFACE_DEPENDENCY, true)
            setProperty(CoreOptions.NO_LAYOUT, true) // no routing!
            DiagramSyntheses.suppressSelectability(it)
            
            addPolyline() => [
                lineStyle = LineStyle.DOT
                noSelectionStyle()
                
                invisible = expanded // make sure there is a style to toggle!
            ]
        ]
    }
}
