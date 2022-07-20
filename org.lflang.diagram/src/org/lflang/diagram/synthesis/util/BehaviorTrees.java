/*************
* Copyright (c) 2022, Kiel University.
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


import java.util.EnumSet;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.List;

import org.eclipse.elk.alg.layered.options.EdgeStraighteningStrategy;
import org.eclipse.elk.alg.layered.options.FixedAlignment;
import org.eclipse.elk.alg.layered.options.LayeredOptions;
import org.eclipse.elk.alg.layered.options.NodePlacementStrategy;
import org.eclipse.elk.alg.layered.options.OrderingStrategy;
import org.eclipse.elk.core.math.ElkPadding;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.options.Direction;
import org.eclipse.elk.core.options.EdgeRouting;
import org.eclipse.elk.core.options.SizeConstraint;
import org.eclipse.xtext.xbase.lib.Extension;
import org.lflang.AttributeUtils;
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions;
import org.lflang.diagram.synthesis.LinguaFrancaSynthesis;
import org.lflang.diagram.synthesis.styles.LinguaFrancaStyleExtensions;
import org.lflang.generator.ReactorInstance;

import com.google.inject.Inject;

import de.cau.cs.kieler.klighd.SynthesisOption;
import de.cau.cs.kieler.klighd.kgraph.KEdge;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.krendering.KPolyline;
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared;
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KEdgeExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KLabelExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KNodeExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KPolylineExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions;
import de.cau.cs.kieler.klighd.syntheses.DiagramSyntheses;

/**
 * Detect and convert (mimic) behavior trees in Lingua Franca diagrams.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
@ViewSynthesisShared
public class BehaviorTrees extends AbstractSynthesisExtensions {
    
    // BT node type
    enum NodeType {
        ACTION, CONDITION,
        SEQUENCE, FALLBACK, PARALLEL
    }
    
    // Related synthesis option
    public static final SynthesisOption SHOW_BT = 
            SynthesisOption.createCheckOption("Detect Behavior Trees", false).setCategory(LinguaFrancaSynthesis.APPEARANCE);
        
    @Inject @Extension private KNodeExtensions _kNodeExtensions;
    @Inject @Extension private KEdgeExtensions _kEdgeExtensions;
    @Inject @Extension private KLabelExtensions _kLabelExtensions;
    @Inject @Extension private KPolylineExtensions _kPolylineExtensions;
    @Inject @Extension private KRenderingExtensions _kRenderingExtensions;
    @Inject @Extension private KContainerRenderingExtensions _kContainerRenderingExtensions;
    @Inject @Extension private LinguaFrancaStyleExtensions _linguaFrancaStyleExtensions;
    @Inject @Extension private UtilityExtensions _utilityExtensions;

    public void handleBehaviorTrees(List<KNode> nodes, ReactorInstance reactor) {
        if (getBooleanValue(SHOW_BT)) {
            if (getBTNodeType(reactor) != null && getBTNodeType(reactor.getParent()) == null) { // If this is the top most BT node -> transform to tree
                // Create new nodes for reactors
                var btNodes = new LinkedHashMap<ReactorInstance, KNode>(); // New nodes
                var visit = new LinkedList<ReactorInstance>();
                visit.add(reactor);
                while(!visit.isEmpty()) {
                    var r = visit.pop();
                    var t = getBTNodeType(r);
                    if (t != null) {
                        btNodes.put(r, createBTNode(r, t));
                        visit.addAll(r.children);
                    }
                }
                
                // Connect nodes
                for(var entry : btNodes.entrySet()) {
                    var parent = entry.getValue();
                    for (var childReactor : entry.getKey().children) {
                        var child = btNodes.get(childReactor);
                        if (child != null) {
                            KEdge edge = _kEdgeExtensions.createEdge();
                            edge.setSource(parent);
                            edge.setTarget(child);
                            KPolyline line = _kEdgeExtensions.addPolyline(edge);
                            _linguaFrancaStyleExtensions.boldLineSelectionStyle(line);
                            _kPolylineExtensions.addHeadArrowDecorator(line);
                        }
                    }
                }
                
                // Container node
                var container = _kNodeExtensions.createNode();
                _kRenderingExtensions.addInvisibleContainerRendering(container);
                container.getChildren().addAll(btNodes.values()); // Add new content
                
                // Layout
                DiagramSyntheses.setLayoutOption(container, CoreOptions.ALGORITHM, LayeredOptions.ALGORITHM_ID);
                DiagramSyntheses.setLayoutOption(container, CoreOptions.DIRECTION, Direction.DOWN);
                DiagramSyntheses.setLayoutOption(container, LayeredOptions.NODE_PLACEMENT_STRATEGY, NodePlacementStrategy.BRANDES_KOEPF);
                DiagramSyntheses.setLayoutOption(container, LayeredOptions.NODE_PLACEMENT_BK_FIXED_ALIGNMENT, FixedAlignment.BALANCED);
                DiagramSyntheses.setLayoutOption(container, LayeredOptions.NODE_PLACEMENT_BK_EDGE_STRAIGHTENING, EdgeStraighteningStrategy.IMPROVE_STRAIGHTNESS);
                DiagramSyntheses.setLayoutOption(container, CoreOptions.EDGE_ROUTING, EdgeRouting.POLYLINE);
                DiagramSyntheses.setLayoutOption(container, LayeredOptions.CONSIDER_MODEL_ORDER_STRATEGY, OrderingStrategy.NODES_AND_EDGES);
                DiagramSyntheses.setLayoutOption(container, CoreOptions.PADDING, new ElkPadding(0));
                
                // Replace content
                nodes.clear();
                nodes.add(container); 
            }
        }
    }
    
    private NodeType getBTNodeType(ReactorInstance reactor) {
        try {
            var typeName = AttributeUtils.findAttribute(reactor.reactorDefinition, "btnode");
            return NodeType.valueOf(typeName.toUpperCase());
        } catch (Exception e) {
            return null;
        }
    }
    
    private KNode createBTNode(ReactorInstance reactor, NodeType type) {
        var node = _kNodeExtensions.createNode();
        DiagramSyntheses.setLayoutOption(node, CoreOptions.NODE_SIZE_CONSTRAINTS, EnumSet.of(SizeConstraint.MINIMUM_SIZE, SizeConstraint.NODE_LABELS));
        _kNodeExtensions.setMinimalNodeSize(node, 25, 25);
        associateWith(node, reactor.getDefinition().getReactorClass());
        _utilityExtensions.setID(node, reactor.uniqueID());
        switch(type) {
            case ACTION:
                _kRenderingExtensions.addRectangle(node);
                _kLabelExtensions.addInsideCenteredNodeLabel(node, reactor.getDefinition().getReactorClass().getName(), 8);
                break;
            case CONDITION:
                _kRenderingExtensions.addEllipse(node);
                _kLabelExtensions.addInsideCenteredNodeLabel(node, reactor.getDefinition().getReactorClass().getName(), 8);
                break;
            case FALLBACK:
                _kRenderingExtensions.addRectangle(node);
                _kLabelExtensions.addInsideCenteredNodeLabel(node, "?", 8);
                break;
            case PARALLEL:
                _kRenderingExtensions.addRectangle(node);
                _kLabelExtensions.addInsideCenteredNodeLabel(node, "||", 8);
                break;
            case SEQUENCE:
                _kRenderingExtensions.addRectangle(node);
                _kLabelExtensions.addInsideCenteredNodeLabel(node, "\u2192", 8);
                break;
            default:
                return null;
        }
        return node;
    }
}
