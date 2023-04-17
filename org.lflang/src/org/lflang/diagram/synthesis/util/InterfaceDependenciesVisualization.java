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

import com.google.common.collect.ImmutableList;
import com.google.common.collect.Iterables;
import com.google.common.collect.Iterators;
import com.google.common.collect.Sets;
import com.google.inject.Inject;
import de.cau.cs.kieler.klighd.SynthesisOption;
import de.cau.cs.kieler.klighd.kgraph.KEdge;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.kgraph.KPort;
import de.cau.cs.kieler.klighd.krendering.KContainerRendering;
import de.cau.cs.kieler.klighd.krendering.KInvisibility;
import de.cau.cs.kieler.klighd.krendering.KPolyline;
import de.cau.cs.kieler.klighd.krendering.KRectangle;
import de.cau.cs.kieler.klighd.krendering.KRendering;
import de.cau.cs.kieler.klighd.krendering.KRenderingFactory;
import de.cau.cs.kieler.klighd.krendering.LineStyle;
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared;
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KEdgeExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions;
import de.cau.cs.kieler.klighd.syntheses.DiagramSyntheses;
import de.cau.cs.kieler.klighd.util.KlighdProperties;
import java.util.List;
import java.util.Random;
import java.util.Set;
import org.eclipse.elk.core.math.ElkMargin;
import org.eclipse.elk.core.math.Spacing;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.graph.properties.Property;
import org.eclipse.xtext.xbase.lib.Extension;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.eclipse.xtext.xbase.lib.Pair;
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions;
import org.lflang.diagram.synthesis.LinguaFrancaSynthesis;
import org.lflang.diagram.synthesis.styles.LinguaFrancaShapeExtensions;
import org.lflang.diagram.synthesis.styles.LinguaFrancaStyleExtensions;

/**
 * Utility class to handle dependency edges for collapsed reactors in Lingua Franca diagrams.
 * 
 * @author Alexander Schulz-Rosengarten
 */
@ViewSynthesisShared
public class InterfaceDependenciesVisualization extends AbstractSynthesisExtensions {
    
    // Related synthesis option
    public static final SynthesisOption SHOW_INTERFACE_DEPENDENCIES = SynthesisOption.createCheckOption("Port Dependencies in Collapsed Reactors", false).setCategory(LinguaFrancaSynthesis.APPEARANCE);
    
    // Properties for marking diagram elements
    public static final Property<Boolean> INTERFACE_DEPENDENCY = new Property<>("org.lflang.linguafranca.diagram.synthesis.dependency.interface", false);
  
    @Inject @Extension private KEdgeExtensions _kEdgeExtensions;
    @Inject @Extension private KRenderingExtensions _kRenderingExtensions;
    @Inject @Extension private KContainerRenderingExtensions _kContainerRenderingExtensions;
    @Inject @Extension private LinguaFrancaStyleExtensions _linguaFrancaStyleExtensions;
    @Inject @Extension private UtilityExtensions _utilityExtensions;
    
    /**
     * Updates the visibility of interface dependencies edges based on the expansion state.
     */
    public static void updateInterfaceDependencyVisibility(KNode node, boolean expanded) {
        Iterable<KEdge> edges = IterableExtensions.filter(node.getOutgoingEdges(),  it -> {
            return it.getProperty(INTERFACE_DEPENDENCY);
        });
        
        Iterable<Iterable<KRendering>> renders = IterableExtensions.map(edges, (KEdge it) -> {
            return Iterables.<KRendering>filter(it.getData(), KRendering.class);
        });
        
        Iterables.concat(renders).forEach(
            it -> {
                KInvisibility inv = IterableExtensions.last(Iterables.filter(it.getStyles(), KInvisibility.class));
                if (inv == null) {
                    inv = KRenderingFactory.eINSTANCE.createKInvisibility();
                    it.getStyles().add(inv);
                }
                inv.setInvisible(expanded);
            }
        );
    }

    /**
     * Adds interface dependencies to the node if this option is active.
     * Visibility will be adjusted based on expansion state.
     */
    public Spacing addInterfaceDependencies(KNode node, boolean expanded) {
        Spacing marginInit = null;
        if (getBooleanValue(SHOW_INTERFACE_DEPENDENCIES)) {
            List<Pair<KPort, KPort>> deps = getPortDependencies(node);
            if (!deps.isEmpty()) {
                for (Pair<KPort, KPort> pair : deps) {
                    createDependencyEdge(pair, expanded);
                }
                
                // Fix content (label) of collapsed rendering
                KContainerRendering contentContainer = IterableExtensions.findFirst(
                        Iterables.filter(node.getData(), KContainerRendering.class), 
                        it -> { return it.getProperty(KlighdProperties.COLLAPSED_RENDERING); }
                );
                if (contentContainer != null) {
                    if (!contentContainer.getProperty(LinguaFrancaShapeExtensions.REACTOR_CONTENT_CONTAINER)) {
                        contentContainer = IteratorExtensions.findFirst(
                            Iterators.filter(contentContainer.eAllContents(), KContainerRendering.class), 
                            it -> { return it.getProperty(LinguaFrancaShapeExtensions.REACTOR_CONTENT_CONTAINER); }
                        );
                    }
                    if (contentContainer != null) {
                        List<KRendering> content = ImmutableList.copyOf(contentContainer.getChildren());
                        // Put into two new containers such that they are not centered/maximized
                        KRectangle firstContainer = _kContainerRenderingExtensions.addRectangle(contentContainer);
                        _kRenderingExtensions.setInvisible(firstContainer, true);
                        
                        KRectangle secondContainer = _kContainerRenderingExtensions.addRectangle(firstContainer);
                        _kRenderingExtensions.setInvisible(secondContainer, true);
                        _kContainerRenderingExtensions.setGridPlacement(secondContainer, 1);
                        Iterables.addAll(secondContainer.getChildren(), content);
                        _kRenderingExtensions.setPointPlacementData(secondContainer, 
                                _kRenderingExtensions.LEFT, 0, 0.5f, 
                                _kRenderingExtensions.TOP, 0, 0, 
                                _kRenderingExtensions.H_CENTRAL, _kRenderingExtensions.V_TOP, 0, 
                                0, 0, 0);
                      
                        
                        // Adjust ports separate dependency edges from label/content
                        if (content.size() > 0) {
                            marginInit = _utilityExtensions.getPortMarginsInitIfAbsent(node).add(
                                new ElkMargin((content.size() * 20) - 8, 0, 0, 0)
                            );
                        }
                    }
                }
            }
        }
        return marginInit;
    }
    
    /**
     * Find dependencies between ports.
     */
    private List<Pair<KPort, KPort>> getPortDependencies(KNode node) {
        Set<KPort> inputPorts = IterableExtensions.toSet(IterableExtensions.filter(
                node.getPorts(), 
                it -> { return it.getProperty(LinguaFrancaSynthesis.REACTOR_INPUT); })
        );
        Set<KPort> outputPorts = IterableExtensions.toSet(IterableExtensions.filter(
                node.getPorts(), 
                it -> { return it.getProperty(LinguaFrancaSynthesis.REACTOR_OUTPUT); })
        );
        
        // FIXME Replace with real logic
        Random rand = new Random();
        return IterableExtensions.toList(
                IterableExtensions.map(IterableExtensions.filter(
                        Sets.cartesianProduct(inputPorts, outputPorts), 
                        it -> { return rand.nextBoolean(); }), 
                it -> { return new Pair<KPort, KPort>(it.get(0), it.get(1)); }));
    }
    
    /**
     * Create an edges for interface dependencies and adjust visibility based on the expansion state of the node.
     */
    private KEdge createDependencyEdge(final Pair<KPort, KPort> connection, final boolean expanded) {
        KEdge depEdge = _kEdgeExtensions.createEdge();
        depEdge.setSource(connection.getKey().getNode());
        depEdge.setSourcePort(connection.getKey());
        depEdge.setTarget(connection.getValue().getNode());
        depEdge.setTargetPort(connection.getValue());
        depEdge.setProperty(InterfaceDependenciesVisualization.INTERFACE_DEPENDENCY, true);
        depEdge.setProperty(CoreOptions.NO_LAYOUT, true);  // no routing!
        DiagramSyntheses.suppressSelectability(depEdge);
        
        KPolyline polyline = _kEdgeExtensions.addPolyline(depEdge);
        _kRenderingExtensions.setLineStyle(polyline, LineStyle.DOT);
        _linguaFrancaStyleExtensions.noSelectionStyle(polyline);
        _kRenderingExtensions.setInvisible(polyline, expanded); // make sure there is a style to toggle!
        
        return depEdge;
    }
}
