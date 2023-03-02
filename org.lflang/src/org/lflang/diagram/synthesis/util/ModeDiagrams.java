/*************
* Copyright (c) 2021, Kiel University.
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


import java.awt.Color;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.stream.Collectors;

import org.eclipse.elk.alg.layered.options.CenterEdgeLabelPlacementStrategy;
import org.eclipse.elk.alg.layered.options.EdgeStraighteningStrategy;
import org.eclipse.elk.alg.layered.options.FixedAlignment;
import org.eclipse.elk.alg.layered.options.LayerConstraint;
import org.eclipse.elk.alg.layered.options.LayeredOptions;
import org.eclipse.elk.alg.layered.options.NodePlacementStrategy;
import org.eclipse.elk.core.math.ElkPadding;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.options.Direction;
import org.eclipse.elk.core.options.EdgeRouting;
import org.eclipse.elk.core.options.PortConstraints;
import org.eclipse.elk.core.options.PortLabelPlacement;
import org.eclipse.elk.core.options.PortSide;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.xtext.xbase.lib.Extension;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.Pair;
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions;
import org.lflang.diagram.synthesis.LinguaFrancaSynthesis;
import org.lflang.diagram.synthesis.action.MemorizingExpandCollapseAction;
import org.lflang.diagram.synthesis.styles.LinguaFrancaShapeExtensions;
import org.lflang.diagram.synthesis.styles.LinguaFrancaStyleExtensions;
import org.lflang.generator.ModeInstance;
import org.lflang.generator.NamedInstance;
import org.lflang.generator.ModeInstance.Transition;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Action;
import org.lflang.lf.Mode;
import org.lflang.lf.ModeTransition;
import org.lflang.lf.Reactor;
import org.lflang.lf.Timer;

import com.google.common.collect.LinkedHashMultimap;
import com.google.inject.Inject;

import de.cau.cs.kieler.klighd.SynthesisOption;
import de.cau.cs.kieler.klighd.kgraph.KEdge;
import de.cau.cs.kieler.klighd.kgraph.KIdentifier;
import de.cau.cs.kieler.klighd.kgraph.KLabel;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.kgraph.KPort;
import de.cau.cs.kieler.klighd.krendering.Colors;
import de.cau.cs.kieler.klighd.krendering.HorizontalAlignment;
import de.cau.cs.kieler.klighd.krendering.KContainerRendering;
import de.cau.cs.kieler.klighd.krendering.KDecoratorPlacementData;
import de.cau.cs.kieler.klighd.krendering.KEllipse;
import de.cau.cs.kieler.klighd.krendering.KPolyline;
import de.cau.cs.kieler.klighd.krendering.KRectangle;
import de.cau.cs.kieler.klighd.krendering.KRendering;
import de.cau.cs.kieler.klighd.krendering.KRenderingFactory;
import de.cau.cs.kieler.klighd.krendering.KText;
import de.cau.cs.kieler.klighd.krendering.LineStyle;
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared;
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KEdgeExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KLabelExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KNodeExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KPolylineExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KPortExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions;
import de.cau.cs.kieler.klighd.labels.decoration.ITextRenderingProvider;
import de.cau.cs.kieler.klighd.labels.decoration.LabelDecorationConfigurator;
import de.cau.cs.kieler.klighd.labels.decoration.LinesDecorator;
import de.cau.cs.kieler.klighd.labels.decoration.RectangleDecorator;
import de.cau.cs.kieler.klighd.syntheses.DiagramSyntheses;
import de.cau.cs.kieler.klighd.util.KlighdProperties;

/**
 * Transformations to support modes in the Lingua Franca diagram synthesis.
 * 
 * @author Alexander Schulz-Rosengarten
 */
@ViewSynthesisShared
public class ModeDiagrams extends AbstractSynthesisExtensions {
    
    // Related synthesis option
    public static final SynthesisOption MODES_CATEGORY = 
            SynthesisOption.createCategory("Modes", false).setCategory(LinguaFrancaSynthesis.APPEARANCE);
    public static final SynthesisOption SHOW_TRANSITION_LABELS = 
            SynthesisOption.createCheckOption("Transition Labels", true).setCategory(MODES_CATEGORY);
    public static final SynthesisOption INITIALLY_COLLAPSE_MODES = 
            SynthesisOption.createCheckOption("Initially Collapse Modes", true).setCategory(MODES_CATEGORY);
    
    private static final Colors MODE_FG = Colors.SLATE_GRAY;
    private static final Colors MODE_BG = Colors.SLATE_GRAY_3;
    private static final int MODE_BG_ALPHA = 50;
    
    @Inject @Extension private KNodeExtensions _kNodeExtensions;
    @Inject @Extension private KEdgeExtensions _kEdgeExtensions;
    @Inject @Extension private KPortExtensions _kPortExtensions;
    @Inject @Extension private KLabelExtensions _kLabelExtensions;
    @Inject @Extension private KPolylineExtensions _kPolylineExtensions;
    @Inject @Extension private KRenderingExtensions _kRenderingExtensions;
    @Inject @Extension private KContainerRenderingExtensions _kContainerRenderingExtensions;
    @Inject @Extension private LinguaFrancaShapeExtensions _linguaFrancaShapeExtensions;
    @Inject @Extension private LinguaFrancaStyleExtensions _linguaFrancaStyleExtensions;
    @Inject @Extension private UtilityExtensions _utilityExtensions;
    @Inject @Extension private LayoutPostProcessing _layoutPostProcessing;
        
    @Extension private KRenderingFactory _kRenderingFactory = KRenderingFactory.eINSTANCE;
    
    public void handleModes(List<KNode> nodes, ReactorInstance reactor) {
        if (!reactor.modes.isEmpty()) {
            var modeNodes = new LinkedHashMap<ModeInstance, KNode>();
            var modeDefinitionMap = new LinkedHashMap<Mode, ModeInstance>();
            for (ModeInstance mode : reactor.modes) {
                var node = _kNodeExtensions.createNode();
                associateWith(node, mode.getDefinition());
                NamedInstanceUtil.linkInstance(node, mode);
                _utilityExtensions.setID(node, mode.uniqueID());
                
                modeNodes.put(mode, node);
                modeDefinitionMap.put(mode.getDefinition(), mode);
                
                // Layout
                if (mode.isInitial()) {
                    DiagramSyntheses.setLayoutOption(node, LayeredOptions.LAYERING_LAYER_CONSTRAINT, LayerConstraint.FIRST);
                }
                // Use general layout configuration of reactors
                this.<LinguaFrancaSynthesis>getRootSynthesis().configureReactorNodeLayout(node, false); 
                _layoutPostProcessing.configureReactor(node);
                // Adjust for modes
                DiagramSyntheses.setLayoutOption(node, CoreOptions.PORT_CONSTRAINTS, PortConstraints.FREE);
                
                var expansionState = MemorizingExpandCollapseAction.getExpansionState(mode);
                DiagramSyntheses.setLayoutOption(node, KlighdProperties.EXPAND, 
                        expansionState != null ? expansionState : !this.getBooleanValue(INITIALLY_COLLAPSE_MODES));
                
                // Expanded Rectangle
                var expandFigure = addModeFigure(node, mode, true);
                expandFigure.setProperty(KlighdProperties.EXPANDED_RENDERING, true);
                _kRenderingExtensions.addDoubleClickAction(expandFigure, MemorizingExpandCollapseAction.ID);
                
                if (getBooleanValue(LinguaFrancaSynthesis.SHOW_HYPERLINKS)) {
                    // Collapse button
                    KText textButton = _linguaFrancaShapeExtensions.addTextButton(expandFigure, LinguaFrancaSynthesis.TEXT_HIDE_ACTION);
                    _kRenderingExtensions.to(
                            _kRenderingExtensions.from(_kRenderingExtensions.setGridPlacementData(textButton),
                                                       _kRenderingExtensions.LEFT, 8, 0, _kRenderingExtensions.TOP, 0, 0),
                            _kRenderingExtensions.RIGHT, 8, 0, _kRenderingExtensions.BOTTOM, 0, 0);
                    _kRenderingExtensions.addSingleClickAction(textButton, MemorizingExpandCollapseAction.ID);
                    _kRenderingExtensions.addDoubleClickAction(textButton, MemorizingExpandCollapseAction.ID);
                }
                
                if (getBooleanValue(LinguaFrancaSynthesis.SHOW_STATE_VARIABLES)) {
                    // Add mode-local state variables
                    var variables = mode.getDefinition().getStateVars();
                    if (!variables.isEmpty()) {
                        KRectangle rectangle = _kContainerRenderingExtensions.addRectangle(expandFigure);
                        _kRenderingExtensions.setInvisible(rectangle, true);
                        if (!getBooleanValue(LinguaFrancaSynthesis.SHOW_HYPERLINKS)) {
                            _kRenderingExtensions.to(
                                    _kRenderingExtensions.from(
                                            _kRenderingExtensions.setGridPlacementData(rectangle), 
                                            _kRenderingExtensions.LEFT, 6, 0, 
                                            _kRenderingExtensions.TOP, 0, 0), 
                                    _kRenderingExtensions.RIGHT, 6, 0, 
                                    _kRenderingExtensions.BOTTOM, 4, 0);
                        } else {
                            _kRenderingExtensions.to(
                                    _kRenderingExtensions.from(
                                            _kRenderingExtensions.setGridPlacementData(rectangle), 
                                            _kRenderingExtensions.LEFT, 6, 0, 
                                            _kRenderingExtensions.TOP, 4, 0), 
                                    _kRenderingExtensions.RIGHT, 6, 0, 
                                    _kRenderingExtensions.BOTTOM, 0, 0);
                        }
                        _kRenderingExtensions.setHorizontalAlignment(rectangle, HorizontalAlignment.LEFT);
                        this.<LinguaFrancaSynthesis>getRootSynthesis().addStateVariableList(rectangle, variables);
                    }
                }
                
                _kContainerRenderingExtensions.addChildArea(expandFigure);
    
                // Collapse Rectangle
                var collapseFigure = addModeFigure(node, mode, false);
                collapseFigure.setProperty(KlighdProperties.COLLAPSED_RENDERING, true);
                if (this.hasContent(mode)) {
                    _kRenderingExtensions.addDoubleClickAction(collapseFigure, MemorizingExpandCollapseAction.ID);
                    
                    if (getBooleanValue(LinguaFrancaSynthesis.SHOW_HYPERLINKS)) {
                        // Expand button
                        KText textButton = _linguaFrancaShapeExtensions.addTextButton(collapseFigure, LinguaFrancaSynthesis.TEXT_SHOW_ACTION);
                        _kRenderingExtensions.to(_kRenderingExtensions.from(
                                    _kRenderingExtensions.setGridPlacementData(textButton), 
                                    _kRenderingExtensions.LEFT, 8, 0, _kRenderingExtensions.TOP, 0, 0), 
                                _kRenderingExtensions.RIGHT, 8, 0, _kRenderingExtensions.BOTTOM, 8, 0);
                        _kRenderingExtensions.addSingleClickAction(textButton, MemorizingExpandCollapseAction.ID);
                        _kRenderingExtensions.addDoubleClickAction(textButton, MemorizingExpandCollapseAction.ID);
                    }
                }
            }
            
            var modeChildren =  LinkedHashMultimap.<ModeInstance, KNode>create();
            var nodeModes =  new HashMap<KNode, ModeInstance>();
            for (var node : nodes) {
                var instance = NamedInstanceUtil.getLinkedInstance(node);
                if (instance == null && node.getProperty(CoreOptions.COMMENT_BOX)) {
                    var firstEdge = IterableExtensions.<KEdge>head(node.getOutgoingEdges());
                    if (firstEdge != null && firstEdge.getTarget() != null) {
                        instance = NamedInstanceUtil.getLinkedInstance(firstEdge.getTarget());
                    }
                }
                if (instance != null) {
                    var mode = instance.getMode(true);
                    modeChildren.put(mode, node);
                    nodeModes.put(node, mode);
                } else {
                    modeChildren.put(null, node);
                }
            }
            
            var modeContainer = _kNodeExtensions.createNode();
            modeContainer.getChildren().addAll(modeNodes.values());
            var modeContainerFigure = addModeContainerFigure(modeContainer);
            _kRenderingExtensions.addDoubleClickAction(modeContainerFigure, MemorizingExpandCollapseAction.ID);
            
            // Use general layout configuration of reactors
            this.<LinguaFrancaSynthesis>getRootSynthesis().configureReactorNodeLayout(modeContainer, false); 
            _layoutPostProcessing.configureReactor(modeContainer);
            // Adjust for state machine style
            // Create alternating directions to make the model more compact
            DiagramSyntheses.setLayoutOption(modeContainer, CoreOptions.DIRECTION, Direction.DOWN);
            // More state machine like node placement
            DiagramSyntheses.setLayoutOption(modeContainer, LayeredOptions.NODE_PLACEMENT_STRATEGY, NodePlacementStrategy.BRANDES_KOEPF);
            DiagramSyntheses.setLayoutOption(modeContainer, LayeredOptions.NODE_PLACEMENT_BK_FIXED_ALIGNMENT, FixedAlignment.BALANCED);
            DiagramSyntheses.setLayoutOption(modeContainer, LayeredOptions.NODE_PLACEMENT_BK_EDGE_STRAIGHTENING, EdgeStraighteningStrategy.IMPROVE_STRAIGHTNESS);
            // Splines
            DiagramSyntheses.setLayoutOption(modeContainer, CoreOptions.EDGE_ROUTING, EdgeRouting.SPLINES);
            DiagramSyntheses.setLayoutOption(modeContainer, LayeredOptions.EDGE_LABELS_CENTER_LABEL_PLACEMENT_STRATEGY, CenterEdgeLabelPlacementStrategy.TAIL_LAYER);
            DiagramSyntheses.setLayoutOption(modeContainer, CoreOptions.SPACING_NODE_SELF_LOOP, 18.0);
            // Unreachable states are unlikely
            DiagramSyntheses.setLayoutOption(modeContainer, CoreOptions.SEPARATE_CONNECTED_COMPONENTS, false);
            // Equal padding
            DiagramSyntheses.setLayoutOption(modeContainer, CoreOptions.PADDING, new ElkPadding(6));
            if (reactor.modes.stream().anyMatch(m -> m.transitions.stream().anyMatch(t -> t.type == ModeTransition.HISTORY))) {
                // Make additional space for history indicator
                DiagramSyntheses.setLayoutOption(modeContainer, LayeredOptions.SPACING_NODE_NODE_BETWEEN_LAYERS, 
                        modeContainer.getProperty(LayeredOptions.SPACING_NODE_NODE_BETWEEN_LAYERS) 
                        + (getBooleanValue(SHOW_TRANSITION_LABELS) ? 6.0 : 10.0));
            }

            var modeContainerPorts = new HashMap<KPort, KPort>();
            for (var mode : reactor.modes) {
                var modeNode = modeNodes.get(mode);
                var edges = new LinkedHashSet<KEdge>();
                // add children
                for (var child : modeChildren.get(mode)) {
                    nodes.remove(child);
                    modeNode.getChildren().add(child);
                    
                    edges.addAll(child.getIncomingEdges());
                    edges.addAll(child.getOutgoingEdges());
                }
                
                // add transitions
                var representedTargets = new HashSet<Pair<ModeInstance, ModeTransition>>();
                for (var transition : mode.transitions) {
                    if (!representedTargets.contains(new Pair<ModeInstance, ModeTransition>(transition.target, transition.type))) {
                        var edge = _kEdgeExtensions.createEdge();
                        edge.setSource(modeNode);
                        edge.setTarget(modeNodes.get(transition.target));
                        addTransitionFigure(edge, transition);
                        
                        if (getBooleanValue(SHOW_TRANSITION_LABELS)) {
                            associateWith(edge, transition.getDefinition());
                        } else {
                            // Bundle similar transitions
                            representedTargets.add(new Pair<ModeInstance, ModeTransition>(transition.target, transition.type));
                        }
                    }
                }
                
                // handle cross hierarchy edges
                var portCopies = new HashMap<KPort, KPort>();
                var triggerCopies = new HashMap<KNode, KNode>();
                for (var edge : edges) {
                    if (!edge.getProperty(CoreOptions.NO_LAYOUT)) {
                        var sourceNodeMode = nodeModes.get(edge.getSource());
                        if (sourceNodeMode == null) {
                            sourceNodeMode = nodeModes.get(edge.getSource().getParent());
                        }
                        var sourceIsInMode = sourceNodeMode != null;
                        var targetNodeMode = nodeModes.get(edge.getTarget());
                        if (targetNodeMode == null) {
                            targetNodeMode = nodeModes.get(edge.getTarget().getParent());
                        }
                        var targetIsInMode = targetNodeMode != null;
                        
                        if (!sourceIsInMode || !targetIsInMode) {
                            var node = sourceIsInMode ? edge.getTarget() : edge.getSource();
                            
                            if (node.getProperty(LinguaFrancaSynthesis.REACTION_SPECIAL_TRIGGER)) {
                                // Duplicate trigger node
                                if (!triggerCopies.containsKey(node)) {
                                    var copy = EcoreUtil.copy(node);
                                    modeNode.getChildren().add(modeNode.getChildren().indexOf(edge.getTarget()), copy);
                                    triggerCopies.put(node, copy);
                                    
                                    // Adjust copy
                                    copy.getOutgoingEdges().forEach(e -> {e.setTarget(null);e.setTargetPort(null);});
                                    copy.getOutgoingEdges().clear();
                                    copy.getData().stream().filter(d -> d instanceof KIdentifier).forEach(d -> {
                                        var kid = (KIdentifier) d;
                                        kid.setId(kid.getId() + "_" + mode.getName());
                                    });
                                }
                                
                                var newNode = triggerCopies.get(node);
                                edge.setSource(newNode);
                                
                                // Remove trigger on top level if only used in modes
                                if (node.getOutgoingEdges().isEmpty()) {
                                    nodes.remove(node);
                                }
                            } else {
                                var port = sourceIsInMode ? edge.getTargetPort() : edge.getSourcePort();
                                var isLocal = modeChildren.get(null).contains(node);
                                if (isLocal) {
                                    // Add port to mode container
                                    if (modeContainerPorts.containsKey(port)) {
                                        node = modeContainer;
                                        port = modeContainerPorts.get(port);
                                    } else {
                                        var containerPort = _kPortExtensions.createPort();
                                        modeContainerPorts.put(port, containerPort);
                                        modeContainer.getPorts().add(containerPort);
                                        
                                        _kPortExtensions.setPortSize(containerPort, 8, 8);
                                        KRectangle rect = _kRenderingExtensions.addRectangle(containerPort);
                                        _kRenderingExtensions.setPointPlacementData(rect,
                                                _kRenderingExtensions.LEFT, 0, 0.5f,
                                                _kRenderingExtensions.BOTTOM, 0, 0.5f,
                                                _kRenderingExtensions.H_CENTRAL, _kRenderingExtensions.V_CENTRAL,
                                                0, 0, 8, 4);
                                        _kRenderingExtensions.setBackground(rect, Colors.BLACK);
                                        _linguaFrancaStyleExtensions.boldLineSelectionStyle(rect);
                                        
                                        DiagramSyntheses.setLayoutOption(containerPort, CoreOptions.PORT_BORDER_OFFSET, -4.0);
                                        DiagramSyntheses.setLayoutOption(containerPort, CoreOptions.PORT_SIDE, sourceIsInMode ? PortSide.EAST : PortSide.WEST);
                                        
                                        var source = _utilityExtensions.sourceElement(node);
                                        var label = "";
                                        if (source instanceof Action) {
                                            label = ((Action) source).getName();
                                        } else if (source instanceof Timer) {
                                            label = ((Timer) source).getName();
                                        } else if (!port.getLabels().isEmpty()) {
                                            label = port.getLabels().get(0).getText();
                                            if (source instanceof Reactor && getBooleanValue(LinguaFrancaSynthesis.SHOW_INSTANCE_NAMES)) {
                                                NamedInstance<?> linkedInstance = NamedInstanceUtil.getLinkedInstance(node);
                                                if (linkedInstance instanceof ReactorInstance) {
                                                    label = ((ReactorInstance) linkedInstance).getName() + "." + label;
                                                }
                                            }
                                        }
                                        var portLabel = _kLabelExtensions.createLabel(containerPort);
                                        portLabel.setText(label);
                                        var portLabelKText = _kRenderingFactory.createKText();
                                        _kRenderingExtensions.setFontSize(portLabelKText, 8);
                                        portLabel.getData().add(portLabelKText);
                                        
                                        // new connection
                                        var copy = EcoreUtil.copy(edge);
                                        if (sourceIsInMode) {
                                            copy.setSource(modeContainer);
                                            copy.setSourcePort(containerPort);
                                            copy.setTarget(edge.getTarget());
                                        } else {
                                            copy.setTarget(modeContainer);
                                            copy.setTargetPort(containerPort);
                                            copy.setSource(edge.getSource());
                                        }
                                        
                                        node = modeContainer;
                                        port = containerPort;
                                    }
                                }
                                
                                // Duplicate port
                                if (!portCopies.containsKey(port)) {
                                    var copy = EcoreUtil.copy(port);
                                    portCopies.put(port, copy);
                                    
                                    var dummyNode = _kNodeExtensions.createNode();
                                    var newID = mode.uniqueID() + "_";
                                    if (!port.getLabels().isEmpty()) {
                                        newID += IterableExtensions.head(port.getLabels()).getText();
                                    }
                                    _utilityExtensions.setID(dummyNode, newID);
                                    _kRenderingExtensions.addInvisibleContainerRendering(dummyNode);
                                    dummyNode.getPorts().add(copy);
                                    // Assign layer
                                    DiagramSyntheses.setLayoutOption(dummyNode, LayeredOptions.LAYERING_LAYER_CONSTRAINT,
                                            port.getProperty(CoreOptions.PORT_SIDE) == PortSide.WEST ? LayerConstraint.FIRST : LayerConstraint.LAST);
                                    // Configure port spacing
                                    DiagramSyntheses.setLayoutOption(dummyNode, CoreOptions.PORT_LABELS_PLACEMENT, EnumSet.of(PortLabelPlacement.ALWAYS_OTHER_SAME_SIDE, PortLabelPlacement.OUTSIDE));
                                    // Place freely
                                    DiagramSyntheses.setLayoutOption(dummyNode, LayeredOptions.CONSIDER_MODEL_ORDER_NO_MODEL_ORDER, true);
                                    // Switch port side
                                    DiagramSyntheses.setLayoutOption(copy, CoreOptions.PORT_SIDE, 
                                            port.getProperty(CoreOptions.PORT_SIDE) == PortSide.WEST ? PortSide.EAST : PortSide.WEST);
                                    
                                    modeNode.getChildren().add(dummyNode);
                                }
                                var newPort = portCopies.get(port);
                                if (sourceIsInMode) {
                                    edge.setTarget(newPort.getNode());
                                    edge.setTargetPort(newPort);
                                } else {
                                    edge.setSource(newPort.getNode());
                                    edge.setSourcePort(newPort);
                                }
                            }
                        }
                    }
                }
            }
            
            // If mode container is unused (no ports for local connections) -> hide it
            if (modeContainer.getPorts().isEmpty()) {
                _kRenderingExtensions.setInvisible(modeContainerFigure, true);
                DiagramSyntheses.setLayoutOption(modeContainer, CoreOptions.PADDING, new ElkPadding(2));
            } else if (getBooleanValue(LinguaFrancaSynthesis.SHOW_INSTANCE_NAMES)) {
                // Remove mode container port labels of ports representing internal connections
                // because their association to reactor instances is unambiguous due to instance names
                for (var p : modeContainer.getPorts()) {
                    p.getLabels().removeIf(l -> l.getText().contains("."));
                }
            }
            
            nodes.add(modeContainer);
        }
    }
    
    private boolean hasContent(ModeInstance mode) {
        return !mode.reactions.isEmpty() || !mode.instantiations.isEmpty();
    }
    
    private KContainerRendering addModeFigure(KNode node, ModeInstance mode, boolean expanded) {
        int padding = getBooleanValue(LinguaFrancaSynthesis.SHOW_HYPERLINKS) ? 8 : 6;
        
        var figure = _kRenderingExtensions.addRoundedRectangle(node, 13, 13, 1);
        _kContainerRenderingExtensions.setGridPlacement(figure, 1);
        _kRenderingExtensions.setLineWidth(figure, mode.isInitial() ? 3f : 1.5f);
        _kRenderingExtensions.setForeground(figure, MODE_FG);
        _kRenderingExtensions.setBackground(figure, MODE_BG);
        var background = _kRenderingExtensions.getBackground(figure);
        background.setAlpha(MODE_BG_ALPHA);
        _linguaFrancaStyleExtensions.boldLineSelectionStyle(figure);
        
        // Invisible container
        KRectangle container = _kContainerRenderingExtensions.addRectangle(figure);
        _kRenderingExtensions.setInvisible(container, true);
        int bottomPadding = this.hasContent(mode) && expanded ? 4 : padding;
        var from = _kRenderingExtensions.from(
                _kRenderingExtensions.setGridPlacementData(container),
                _kRenderingExtensions.LEFT, padding, 0, _kRenderingExtensions.TOP, padding, 0);
        _kRenderingExtensions.to(from, _kRenderingExtensions.RIGHT, padding, 0, _kRenderingExtensions.BOTTOM, bottomPadding, 0);
        
        // Centered child container
        KRectangle childContainer = _kContainerRenderingExtensions.addRectangle(container);
        this._kRenderingExtensions.setInvisible(childContainer, true);
        this._kRenderingExtensions.setPointPlacementData(childContainer,
                _kRenderingExtensions.LEFT, 0, 0.5f, _kRenderingExtensions.TOP, 0, 0.5f,
                _kRenderingExtensions.H_CENTRAL, _kRenderingExtensions.V_CENTRAL, 0, 0, 0, 0);
        this._kContainerRenderingExtensions.setGridPlacement(childContainer, 1);
        
        KText text = _kContainerRenderingExtensions.addText(childContainer, mode.getName());
        DiagramSyntheses.suppressSelectability(text);
        _linguaFrancaStyleExtensions.underlineSelectionStyle(text);
        
        return figure;
    }
    
    private KContainerRendering addModeContainerFigure(KNode node) {
        var rect = _kRenderingExtensions.addRectangle(node);
        _kRenderingExtensions.setLineWidth(rect, 1);
        _kRenderingExtensions.setLineStyle(rect, LineStyle.DOT);
        _kRenderingExtensions.setForeground(rect, MODE_FG);
        _linguaFrancaStyleExtensions.boldLineSelectionStyle(rect);
        return rect;
    }
    
    private void addTransitionFigure(KEdge edge, Transition transition) {
        var spline = _kEdgeExtensions.addSpline(edge);
        _kRenderingExtensions.setLineWidth(spline, 1.5f);
        _kRenderingExtensions.setForeground(spline, MODE_FG);
        _linguaFrancaStyleExtensions.boldLineSelectionStyle(spline);

        if (transition.type == ModeTransition.HISTORY) {
            addHistoryDecorator(spline);
        } else {
            KRendering arrowDecorator = _kPolylineExtensions.addHeadArrowDecorator(spline);
            this._kRenderingExtensions.setForeground(arrowDecorator, MODE_FG);
            this._kRenderingExtensions.setBackground(arrowDecorator, MODE_FG);
        }
            
        if (getBooleanValue(SHOW_TRANSITION_LABELS)) {
            associateWith(spline, transition.getDefinition());
            
            KLabel centerEdgeLabel = _kLabelExtensions.addCenterEdgeLabel(edge, this.toTransitionLabel(transition));
            associateWith(centerEdgeLabel, transition.getDefinition());
            applyTransitionOnEdgeStyle(centerEdgeLabel);
        }
    }
    
    private String toTransitionLabel(Transition transition) {
        var text = new StringBuilder();
        
        text.append(transition.reaction.triggers.stream().map(t -> t.getDefinition().getName()).collect(Collectors.joining(", ")));
        return text.toString();
    }
    
    private static LabelDecorationConfigurator _onEdgeTransitionLabelConfigurator; // ONLY for use in applyTransitionOnEdgeStyle
    private void applyTransitionOnEdgeStyle(KLabel label) {
        if (_onEdgeTransitionLabelConfigurator == null) {
            var foreground = new Color(MODE_FG.getRed(), MODE_FG.getGreen(), MODE_FG.getBlue());
            var background = new Color(Colors.GRAY_95.getRed(), Colors.GRAY_95.getGreen(), Colors.GRAY_95.getBlue());
            _onEdgeTransitionLabelConfigurator = LabelDecorationConfigurator.create()
                .withInlineLabels(true)
                .withLabelTextRenderingProvider(new ITextRenderingProvider() {
                    @Override
                    public KRendering createTextRendering(
                            KContainerRendering container, KLabel llabel) {
                        var kText = _kRenderingFactory.createKText();
                        _kRenderingExtensions.setFontSize(kText, 8);
                        container.getChildren().add(kText);
                        return kText;
                    }
                })
                .addDecoratorRenderingProvider(RectangleDecorator.create().withBackground(background))
                .addDecoratorRenderingProvider(LinesDecorator.create().withColor(foreground));
        }
        _onEdgeTransitionLabelConfigurator.applyTo(label);
    }
    
    private void addHistoryDecorator(KPolyline line) {
        var decorator = _kPolylineExtensions.addHeadArrowDecorator(line);
        ((KDecoratorPlacementData) decorator.getPlacementData()).setAbsolute((-15.0f));

        var ellipse = _kContainerRenderingExtensions.addEllipse(line);
        _kRenderingExtensions.<KEllipse>setDecoratorPlacementData(ellipse, 16, 16, (-6), 1, false);
        _kRenderingExtensions.setLineWidth(ellipse, 0.8f);
        _kRenderingExtensions.setForeground(ellipse, MODE_FG);
        _kRenderingExtensions.setBackground(ellipse, Colors.WHITE);
        
        var innerLine = _kContainerRenderingExtensions.addPolyline(ellipse);
        _kRenderingExtensions.setLineWidth(innerLine, 2);
        var points = innerLine.getPoints();
        points.add(_kRenderingExtensions.createKPosition(_kRenderingExtensions.LEFT, 5, 0, _kRenderingExtensions.TOP, 4, 0));
        points.add(_kRenderingExtensions.createKPosition(_kRenderingExtensions.LEFT, 5, 0, _kRenderingExtensions.BOTTOM, 4, 0));
        points.add(_kRenderingExtensions.createKPosition(_kRenderingExtensions.LEFT, 5, 0, _kRenderingExtensions.TOP, 0, 0.5f));
        points.add(_kRenderingExtensions.createKPosition(_kRenderingExtensions.RIGHT, 5, 0, _kRenderingExtensions.TOP, 0, 0.5f));
        points.add(_kRenderingExtensions.createKPosition(_kRenderingExtensions.RIGHT, 5, 0, _kRenderingExtensions.BOTTOM, 4, 0));
        points.add(_kRenderingExtensions.createKPosition(_kRenderingExtensions.RIGHT, 5, 0, _kRenderingExtensions.TOP, 4, 0));
        _kRenderingExtensions.setForeground(innerLine, MODE_FG);
    }
    
}