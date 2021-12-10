/*
 * KIELER - Kiel Integrated Environment for Layout Eclipse RichClient
 *
 * http://rtsys.informatik.uni-kiel.de/kieler
 * 
 * Copyright 2021 by
 * + Kiel University
 *   + Department of Computer Science
 *     + Real-Time and Embedded Systems Group
 * 
 * This code is provided under the terms of the Eclipse Public License (EPL).
 */
package org.lflang.diagram.synthesis.util

import com.google.common.collect.HashMultimap
import com.google.inject.Inject
import de.cau.cs.kieler.klighd.SynthesisOption
import de.cau.cs.kieler.klighd.kgraph.KEdge
import de.cau.cs.kieler.klighd.kgraph.KLabel
import de.cau.cs.kieler.klighd.kgraph.KNode
import de.cau.cs.kieler.klighd.krendering.Colors
import de.cau.cs.kieler.klighd.krendering.KContainerRendering
import de.cau.cs.kieler.klighd.krendering.KDecoratorPlacementData
import de.cau.cs.kieler.klighd.krendering.KPolyline
import de.cau.cs.kieler.klighd.krendering.KRenderingFactory
import de.cau.cs.kieler.klighd.krendering.LineStyle
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KEdgeExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KLabelExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KNodeExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KPolylineExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KPortExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions
import de.cau.cs.kieler.klighd.labels.decoration.LabelDecorationConfigurator
import de.cau.cs.kieler.klighd.labels.decoration.LinesDecorator
import de.cau.cs.kieler.klighd.labels.decoration.RectangleDecorator
import de.cau.cs.kieler.klighd.util.KlighdProperties
import java.awt.Color
import java.util.LinkedHashMap
import java.util.List
import org.eclipse.elk.alg.layered.options.LayerConstraint
import org.eclipse.elk.alg.layered.options.LayeredOptions
import org.eclipse.elk.core.math.ElkPadding
import org.eclipse.elk.core.options.CoreOptions
import org.eclipse.elk.core.options.Direction
import org.eclipse.elk.core.options.EdgeRouting
import org.eclipse.elk.core.options.PortConstraints
import org.eclipse.elk.core.options.PortSide
import org.eclipse.elk.core.options.SizeConstraint
import org.eclipse.emf.ecore.util.EcoreUtil
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions
import org.lflang.diagram.synthesis.LinguaFrancaSynthesis
import org.lflang.diagram.synthesis.action.MemorizingExpandCollapseAction
import org.lflang.diagram.synthesis.styles.LinguaFrancaShapeExtensions
import org.lflang.diagram.synthesis.styles.LinguaFrancaStyleExtensions
import org.lflang.generator.ModeInstance
import org.lflang.generator.ModeInstance.Transition
import org.lflang.generator.ReactorInstance
import org.lflang.lf.Action
import org.lflang.lf.ModeTransitionTypes
import org.lflang.lf.Timer
import org.lflang.lf.VarRef

import static org.lflang.diagram.synthesis.LinguaFrancaSynthesis.*

import static extension de.cau.cs.kieler.klighd.syntheses.DiagramSyntheses.*
import static extension org.lflang.diagram.synthesis.action.MemorizingExpandCollapseAction.*
import static extension org.lflang.diagram.synthesis.util.NamedInstanceUtil.*

/**
 * @author als
 */
@ViewSynthesisShared
class ModeDiagrams extends AbstractSynthesisExtensions {
    
    // Related synthesis option
    public static val SynthesisOption MODES_CATEGORY = SynthesisOption.createCategory("Modes", false).setCategory(LinguaFrancaSynthesis.APPEARANCE)
    //public static val SynthesisOption BREAK_CONNECTIONS = SynthesisOption.createCheckOption("Partition Connections into Modes", true).setCategory(MODES_CATEGORY)
    public static val SynthesisOption SHOW_TRANSITION_LABELS = SynthesisOption.createCheckOption("Transition Labels", true).setCategory(MODES_CATEGORY)
    public static val SynthesisOption INITIALLY_COLLAPSE_MODES = SynthesisOption.createCheckOption("Initially Collapse Modes", true).setCategory(MODES_CATEGORY)
    
    private static val MODE_FG = Colors.SLATE_GRAY
    private static val MODE_BG = Colors.SLATE_GRAY_3
    private static val MODE_BG_ALPHA = 50
    
    @Inject extension KNodeExtensions
    @Inject extension KEdgeExtensions
    @Inject extension KPortExtensions
    @Inject extension KLabelExtensions
    @Inject extension KPolylineExtensions
    @Inject extension KRenderingExtensions
    @Inject extension KContainerRenderingExtensions
    @Inject extension LinguaFrancaShapeExtensions
    @Inject extension LinguaFrancaStyleExtensions
    @Inject extension UtilityExtensions
    extension KRenderingFactory = KRenderingFactory::eINSTANCE
    
    def void handleModes(List<KNode> nodes, ReactorInstance reactor) {
        if (!reactor.modes.empty) {
            val modeNodes = new LinkedHashMap()
            val modeDefinitionMap = new LinkedHashMap()
            for (mode : reactor.modes) {
                val node = createNode().associateWith(mode.definition)
                node.linkInstance(mode)
                node.ID = mode.uniqueID
                modeNodes.put(mode, node)
                modeDefinitionMap.put(mode.definition, mode)
                
                if (mode.initial) {
                    node.setLayoutOption(LayeredOptions.LAYERING_LAYER_CONSTRAINT, LayerConstraint.FIRST)
                }
                node.setLayoutOption(LayeredOptions.CROSSING_MINIMIZATION_SEMI_INTERACTIVE, true)
                
                node.setLayoutOption(KlighdProperties.EXPAND, mode.getExpansionState?:(!INITIALLY_COLLAPSE_MODES.booleanValue))
                
                // Expanded Rectangle
                node.addModeFigure(mode, true) => [
                    setProperty(KlighdProperties.EXPANDED_RENDERING, true)
                    addDoubleClickAction(MemorizingExpandCollapseAction.ID)
    
                    if (SHOW_HYPERLINKS.booleanValue) {
                        // Collapse button
                        addTextButton(TEXT_HIDE_ACTION) => [
                            setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 0, 0)
                            addSingleClickAction(MemorizingExpandCollapseAction.ID)
                            addDoubleClickAction(MemorizingExpandCollapseAction.ID)
                        ]
                    }
                    
                    addChildArea()
                ]
    
                // Collapse Rectangle
                node.addModeFigure(mode, false) => [
                    setProperty(KlighdProperties.COLLAPSED_RENDERING, true)
                    if (mode.hasContent) {
                        addDoubleClickAction(MemorizingExpandCollapseAction.ID)
                    }
    
                    if (SHOW_HYPERLINKS.booleanValue) {
                        // Expand button
                        if (mode.hasContent) {
                            addTextButton(TEXT_SHOW_ACTION) => [
                                setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 8, 0)
                                addSingleClickAction(MemorizingExpandCollapseAction.ID)
                                addDoubleClickAction(MemorizingExpandCollapseAction.ID)
                            ]
                        }
                    }
                ]
            }
            
            val modeChildren =  HashMultimap.create
            val nodeModes =  newHashMap
            for (node : nodes) {
                var instance = node.linkedInstance
                if (instance === null && node.getProperty(CoreOptions.COMMENT_BOX)) {
                    instance = node.outgoingEdges.head?.target?.linkedInstance
                }
                if (instance !== null) {
                    val mode = instance.getMode(true)
                    modeChildren.put(mode, node)
                    nodeModes.put(node, mode)
                } else {
                    modeChildren.put(null, node)
                }
            }
            
            val modeContainer = createNode() => [
                children += modeNodes.values
                
                val fig = addModeContainerFigure => [
                    addDoubleClickAction(MemorizingExpandCollapseAction.ID)
                ]
                if (modeChildren.get(null).empty) {
                    fig.invisible = true
                    setLayoutOption(CoreOptions.PADDING, new ElkPadding())
                }
                
                setLayoutOption(CoreOptions.NODE_SIZE_CONSTRAINTS, SizeConstraint.minimumSizeWithPorts)
                setLayoutOption(CoreOptions.EDGE_ROUTING, EdgeRouting.SPLINES)
                setLayoutOption(CoreOptions.DIRECTION, Direction.DOWN)
                setLayoutOption(CoreOptions.PORT_CONSTRAINTS, PortConstraints.FIXED_ORDER)
            ]
            val modeContainerPorts = newHashMap()
            
            for (mode : reactor.modes.reverseView) {
                val modeNode = modeNodes.get(mode)
                val edges = newHashSet
                // add children
                for (child : modeChildren.get(mode)) {
                    nodes.remove(child)
                    modeNode.children.add(child)
                    
                    edges += child.incomingEdges
                    edges += child.outgoingEdges
                }
                
                // add transitions
                val representedTargets = newHashSet
                for (transition : mode.transitions.reverseView) {
                    if (!representedTargets.contains(new Pair(transition.target, transition.type))) {
                        val edge = createEdge()
                        edge.source = modeNode
                        edge.target = modeNodes.get(transition.target)
                        edge.addTransitionFigure(transition)
                        
                        if (SHOW_TRANSITION_LABELS.booleanValue) {
                            edge.associateWith(transition.definition)
                        } else {
                            // Bundle similar transitions
                            representedTargets += new Pair(transition.target, transition.type)
                        }
                    }
                }
                
                // handle cross hierarchy edges
                val portCopies = newHashMap
                for (edge : edges.filter[!it.getProperty(CoreOptions.NO_LAYOUT)]) {
                    val sourceNodeMode = nodeModes.get(edge.source)?:nodeModes.get(edge.source.parent)
                    val sourceIsInMode = sourceNodeMode !== null
                    val targetNodeMode = nodeModes.get(edge.target)?:nodeModes.get(edge.target.parent)
                    val targetIsInMode = targetNodeMode !== null
                    if (!sourceIsInMode || !targetIsInMode) {
                        var node = sourceIsInMode ? edge.target : edge.source
                        var port = sourceIsInMode ? edge.targetPort : edge.sourcePort
                        val isLocal = modeChildren.get(null).contains(node)
                        if (isLocal) {
                            // Add port to mode container
                            if (modeContainerPorts.containsKey(port)) {
                                node = modeContainer
                                port = modeContainerPorts.get(port)
                            } else {
                                val containerPort = createPort()
                                modeContainerPorts.put(port, containerPort)
                                modeContainer.ports += containerPort
                                
                                containerPort.setPortSize(8, 4)
                                containerPort.addRectangle => [
                                    background = Colors.BLACK
                                ]
                                containerPort.setLayoutOption(CoreOptions.PORT_BORDER_OFFSET, -4.0)
                                containerPort.setLayoutOption(CoreOptions.PORT_SIDE, sourceIsInMode ? PortSide.EAST : PortSide.WEST)
                                
                                val source = node.sourceElement
                                val label = switch(source) {
                                    Action: source.name
                                    Timer: source.name
                                    default: ""
                                }
                                containerPort.addOutsidePortLabel(label, 8)
                                
                                // new connection
                                val copy = EcoreUtil.copy(edge)
                                if (sourceIsInMode) {
                                    copy.source = modeContainer
                                    copy.sourcePort = containerPort
                                    copy.target = edge.target
                                } else {
                                    copy.target = modeContainer
                                    copy.targetPort = containerPort
                                    copy.source = edge.source
                                }
                                
                                node = modeContainer
                                port = containerPort
                            }
                        }
                        // Duplicate port
                        if (!portCopies.containsKey(port)) {
                            val copy = EcoreUtil.copy(port)
                            portCopies.put(port, copy)
                            
                            val dummyNode = createNode()
                            dummyNode.ID = mode.uniqueID + "_" + port.labels?.head?.text
                            dummyNode.addInvisibleContainerRendering()
                            dummyNode.ports += copy
                            dummyNode.setLayoutOption(LayeredOptions.LAYERING_LAYER_CONSTRAINT, port.getProperty(CoreOptions.PORT_SIDE) === PortSide.WEST ? LayerConstraint.FIRST : LayerConstraint.LAST)
                            
                            modeNode.children += dummyNode
                        }
                        val newPort = portCopies.get(port)
                        if (sourceIsInMode) {
                            edge.target = newPort.node
                            edge.targetPort = newPort
                        } else {
                            edge.source = newPort.node
                            edge.sourcePort = newPort
                        }
                    }
                }
            }
            
            nodes += modeContainer
        }
    }
    
    def hasContent(ModeInstance mode) {
        return !mode.reactions.empty || !mode.instantiations.empty
    }
    
    def KContainerRendering addModeFigure(KNode node, ModeInstance mode, boolean expanded) {
        val padding = SHOW_HYPERLINKS.booleanValue ? 8 : 6
        val figure = node.addRoundedRectangle(13, 13, 1) => [
            setGridPlacement(1)
            lineWidth = mode.initial ? 3f : 1.5f
            foreground = MODE_FG
            background = MODE_BG
            background.alpha = MODE_BG_ALPHA
            boldLineSelectionStyle
        ]
        
        figure.addRectangle() => [
            invisible = true
            setGridPlacementData().from(LEFT, padding, 0, TOP, padding, 0).to(RIGHT, padding, 0, BOTTOM, (mode.hasContent && expanded) ? 4 : padding, 0)
            
            addRectangle() => [ // Centered child container
                invisible = true
                setPointPlacementData(LEFT, 0, 0.5f, TOP, 0, 0.5f, H_CENTRAL, V_CENTRAL, 0, 0, 0, 0)
                setGridPlacement(1)
                
                addText(mode.name) => [
                    suppressSelectability
                    underlineSelectionStyle
                ]
            ]
        ]
        
        return figure
    }
    
    def KContainerRendering addModeContainerFigure(KNode node) {
        return node.addRectangle => [
            lineWidth = 1
            lineStyle = LineStyle.DOT
            foreground = MODE_FG
            boldLineSelectionStyle()
        ]
    }
    
    def void addTransitionFigure(KEdge edge, Transition transition) {
        val spline = edge.addSpline() => [
            lineWidth = 1.5f
            
            foreground = MODE_FG
            background = MODE_FG
            
            boldLineSelectionStyle()
        ]

        if (transition.type === ModeTransitionTypes.HISTORY) {
            spline.addHistoryDecorator()
        } else {
            spline.addHeadArrowDecorator() => [
                foreground = MODE_FG
                background = MODE_FG
            ]
        }
            
        if (SHOW_TRANSITION_LABELS.booleanValue) {
            spline.associateWith(transition.definition)
            
            edge.addCenterEdgeLabel(transition.toTransitionLabel) => [
                associateWith(transition.definition)
                applyTransitionOnEdgeStyle()
            ]
        }
    }
    
    def toTransitionLabel(Transition transition) {
        val text = new StringBuilder
        
        text.append(transition.reaction.triggers.map[definition.name].join(", "))
        
        if (!transition.reaction.effects.empty) {
            text.append(" / ")
            for(eff : transition.reaction.effects) {
                if (eff.definition instanceof VarRef && (eff.definition as VarRef).container !== null) {
                    text.append((eff.definition as VarRef).container.name).append(".")
                }
                text.append(eff.definition.name)
            }
        }
        return text.toString
    }
    
    static var LabelDecorationConfigurator _onEdgeTransitionLabelConfigurator; // ONLY for use in applyTransitionOnEdgeStyle
    def applyTransitionOnEdgeStyle(KLabel label) {
        if (_onEdgeTransitionLabelConfigurator === null) {
            val foreground = new Color(MODE_FG.red, MODE_FG.green, MODE_FG.blue)
            val background = new Color(Colors.GRAY_95.red, Colors.GRAY_95.green, Colors.GRAY_95.blue)
            _onEdgeTransitionLabelConfigurator = LabelDecorationConfigurator.create
                .withInlineLabels(true)
                .withLabelTextRenderingProvider([ container, klabel | 
                    val kText = createKText()
                    kText.fontSize = 8
                    container.children += kText
                    kText
                ])
                .addDecoratorRenderingProvider(RectangleDecorator.create().withBackground(background))
                .addDecoratorRenderingProvider(LinesDecorator.create().withColor(foreground))
                //.addDecoratorRenderingProvider(DirectionalArrowsDecorator.create().withColor(foreground))
        }
        _onEdgeTransitionLabelConfigurator.applyTo(label)
    }
    
    def addHistoryDecorator(KPolyline line) {
        line.addHeadArrowDecorator() => [
            (placementData as KDecoratorPlacementData).absolute = -15.0f;
        ]
        line.addEllipse() => [
            setDecoratorPlacementData(16, 16, -6, 1, false);
            lineWidth = 0.8f;
            foreground = MODE_FG;
            background = Colors.WHITE;
            addPolyline => [
                lineWidth = 2;
                points += createKPosition(LEFT, 5, 0, TOP, 4, 0);
                points += createKPosition(LEFT, 5, 0, BOTTOM, 4, 0);
                points += createKPosition(LEFT, 5, 0, TOP, 0, 0.5f);
                points += createKPosition(RIGHT, 5, 0, TOP, 0, 0.5f);
                points += createKPosition(RIGHT, 5, 0, BOTTOM, 4, 0);
                points += createKPosition(RIGHT, 5, 0, TOP, 4, 0);
                foreground = MODE_FG;
            ]
        ]
    }
    
}