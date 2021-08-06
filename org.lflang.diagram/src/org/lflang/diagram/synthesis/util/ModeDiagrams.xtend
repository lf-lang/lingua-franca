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
import org.eclipse.elk.graph.properties.Property
import org.eclipse.emf.ecore.EObject
import org.eclipse.emf.ecore.util.EcoreUtil
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions
import org.lflang.diagram.synthesis.LinguaFrancaSynthesis
import org.lflang.diagram.synthesis.styles.LinguaFrancaShapeExtensions
import org.lflang.diagram.synthesis.styles.LinguaFrancaStyleExtensions
import org.lflang.lf.Action
import org.lflang.lf.Mode
import org.lflang.lf.Reaction
import org.lflang.lf.Reactor
import org.lflang.lf.Timer
import org.lflang.lf.VarRef

import static org.lflang.diagram.synthesis.LinguaFrancaSynthesis.*
import static org.lflang.diagram.synthesis.action.MemorizingExpandCollapseAction.*

import static extension de.cau.cs.kieler.klighd.syntheses.DiagramSyntheses.*
import org.lflang.generator.ReactorInstance
import org.lflang.generator.NamedInstance

/**
 * @author als
 */
@ViewSynthesisShared
class ModeDiagrams extends AbstractSynthesisExtensions {
    
    public static val MODE = new Property<Mode>("org.lflang.linguafranca.diagram.synthesis.mode", null)
    
    // Related synthesis option
    public static val SynthesisOption MODES_CATEGORY = SynthesisOption.createCategory("Modes", true).setCategory(LinguaFrancaSynthesis.APPEARANCE)
    //public static val SynthesisOption BREAK_CONNECTIONS = SynthesisOption.createCheckOption("Partition Connections into Modes", true).setCategory(MODES_CATEGORY)
    public static val SynthesisOption SHOW_TRANSITION_LABELS = SynthesisOption.createCheckOption("Transition Labels", true).setCategory(MODES_CATEGORY)
    
    
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
    
//    def void setMode(KNode node, NamedInstance<?> obj) {
//        if (obj.eContainer instanceof Mode) {
//            node.setProperty(MODE, obj.eContainer as Mode)
//        }
//    }
    
    def void handleModes(List<KNode> nodes, ReactorInstance reactor) {
//        if (!reactor.modes.empty) {
//            val modeNodes = new LinkedHashMap()
//            for (mode : reactor.modes) {
//                val node = createNode().associateWith(mode)
//                modeNodes.put(mode, node)
//                
//                if (mode.initial) {
//                    node.setLayoutOption(LayeredOptions.LAYERING_LAYER_CONSTRAINT, LayerConstraint.FIRST)
//                }
//                node.setLayoutOption(LayeredOptions.CROSSING_MINIMIZATION_SEMI_INTERACTIVE, true)
//                
//                // Expanded Rectangle
//                node.addModeFigure(mode, true) => [
//                    setProperty(KlighdProperties.EXPANDED_RENDERING, true)
//                    addDoubleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)
//    
//                    if (SHOW_HYPERLINKS.booleanValue) {
//                        // Collapse button
//                        addTextButton(TEXT_HIDE_ACTION) => [
//                            setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 0, 0)
//                            addSingleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)
//                            addDoubleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)
//                        ]
//                    }
//                    
//                    addChildArea()
//                ]
//    
//                // Collapse Rectangle
//                node.addModeFigure(mode, false) => [
//                    setProperty(KlighdProperties.COLLAPSED_RENDERING, true)
//                    if (mode.hasContent) {
//                        addDoubleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)
//                    }
//    
//                    if (SHOW_HYPERLINKS.booleanValue) {
//                        // Expand button
//                        if (mode.hasContent) {
//                            addTextButton(TEXT_SHOW_ACTION) => [
//                                setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 8, 0)
//                                addSingleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)
//                                addDoubleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)
//                            ]
//                        }
//                    }
//                ]
//            }
//            
//            val modeChildren =  HashMultimap.create
//            for (node : nodes) {
//                modeChildren.put(node.getProperty(MODE), node)
//            }
//            
//            val modeContainer = createNode() => [
//                children += modeNodes.values
//                
//                val fig = addModeContainerFigure
//                if (modeChildren.get(null).empty) {
//                    fig.invisible = true
//                    setLayoutOption(CoreOptions.PADDING, new ElkPadding())
//                }
//                
//                setLayoutOption(CoreOptions.NODE_SIZE_CONSTRAINTS, SizeConstraint.minimumSizeWithPorts)
//                setLayoutOption(CoreOptions.EDGE_ROUTING, EdgeRouting.SPLINES)
//                setLayoutOption(CoreOptions.DIRECTION, Direction.DOWN)
//                setLayoutOption(CoreOptions.PORT_CONSTRAINTS, PortConstraints.FIXED_ORDER)
//            ]
//            val modeContainerPorts = newHashMap()
//            
//            for (mode : reactor.modes.reverseView) {
//                val modeNode = modeNodes.get(mode)
//                val edges = newHashSet
//                // add children
//                for (child : modeChildren.get(mode)) {
//                    nodes.remove(child)
//                    modeNode.children.add(child)
//                    
//                    edges += child.incomingEdges
//                    edges += child.outgoingEdges
//                }
//                // add transitions
//                if (SHOW_TRANSITION_LABELS.booleanValue) {
//                    for (reaction : mode.reactions.reverseView) {
//                        for (targetMode : reaction.effects.map[variable].filter(Mode).toSet) {
//                            createEdge() => [
//                                source = modeNode
//                                target = modeNodes.get(targetMode)
//                                
//                                associateWith(reaction)
//                                addTransitionFigure(reaction)
//                            ]
//                        } 
//                    }
//                } else {
//                    for (targetMode : mode.reactions.reverseView.map[effects].flatten.map[variable].filter(Mode).toSet) {
//                        createEdge() => [
//                            source = modeNode
//                            target = modeNodes.get(targetMode)
//                            addTransitionFigure(null)
//                        ]
//                    }
//                }
//                // handle cross hierarchy edges
//                val portCopies = newHashMap
//                for (edge : edges) {
//                    val sourceNodeMode = edge.source.getProperty(MODE)
//                    val sourceIsInMode = sourceNodeMode !== null && modeNodes.containsKey(sourceNodeMode)
//                    val targetNodeMode = edge.target.getProperty(MODE)
//                    val targetIsInMode = targetNodeMode !== null && modeNodes.containsKey(targetNodeMode)
//                    if (!sourceIsInMode || !targetIsInMode) {
//                        var node = sourceIsInMode ? edge.target : edge.source
//                        var port = sourceIsInMode ? edge.targetPort : edge.sourcePort
//                        val isLocal = modeChildren.get(null).contains(node)
//                        if (isLocal) {
//                            // Add port to mode container
//                            if (modeContainerPorts.containsKey(port)) {
//                                node = modeContainer
//                                port = modeContainerPorts.get(port)
//                            } else {
//                                val containerPort = createPort()
//                                modeContainerPorts.put(port, containerPort)
//                                modeContainer.ports += containerPort
//                                
//                                containerPort.setPortSize(8, 4)
//                                containerPort.addRectangle => [
//                                    background = Colors.BLACK
//                                ]
//                                containerPort.setLayoutOption(CoreOptions.PORT_BORDER_OFFSET, -4.0)
//                                containerPort.setLayoutOption(CoreOptions.PORT_SIDE, sourceIsInMode ? PortSide.EAST : PortSide.WEST)
//                                
//                                val source = node.sourceElement
//                                val label = switch(source) {
//                                    Action: source.name
//                                    Timer: source.name
//                                    default: ""
//                                }
//                                containerPort.addOutsidePortLabel(label, 8)
//                                
//                                // new connection
//                                val copy = EcoreUtil.copy(edge)
//                                if (sourceIsInMode) {
//                                    copy.source = modeContainer
//                                    copy.sourcePort = containerPort
//                                    copy.target = copy.targetPort.node
//                                } else {
//                                    copy.target = modeContainer
//                                    copy.targetPort = containerPort
//                                    copy.source = copy.sourcePort.node
//                                }
//                                
//                                node = modeContainer
//                                port = containerPort
//                            }
//                        }
//                        // Duplicate port
//                        if (!portCopies.containsKey(port)) {
//                            val copy = EcoreUtil.copy(port)
//                            portCopies.put(port, copy)
//                            
//                            val dummyNode = createNode()
//                            dummyNode.addInvisibleContainerRendering()
//                            dummyNode.ports += copy
//                            dummyNode.setLayoutOption(LayeredOptions.LAYERING_LAYER_CONSTRAINT, port.getProperty(CoreOptions.PORT_SIDE) === PortSide.WEST ? LayerConstraint.FIRST : LayerConstraint.LAST)
//                            
//                            modeNode.children += dummyNode
//                            // TODO adjust appearance
//                        }
//                        val newPort = portCopies.get(port)
//                        if (sourceIsInMode) {
//                            edge.target = newPort.node
//                            edge.targetPort = newPort
//                        } else {
//                            edge.source = newPort.node
//                            edge.sourcePort = newPort
//                        }
//                    }
//                }
//            }
//            
//            nodes += modeContainer
//        }
    }
    
    def hasContent(Mode mode) {
        return !mode.reactions.empty || !mode.instantiations.empty
    }
    
    def KContainerRendering addModeFigure(KNode node, Mode mode, boolean expanded) {
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
    
    def void addTransitionFigure(KEdge edge, Reaction reaction) {
        val spline = edge.addSpline() => [
            lineWidth = 1.5f
            
            foreground = MODE_FG
            foreground.propagateToChildren = true
            background = MODE_FG
            background.propagateToChildren = true
            
            boldLineSelectionStyle()
            addHeadArrowDecorator()
        ]
        if (reaction !== null) {
            spline.associateWith(reaction)
            
            edge.addCenterEdgeLabel(reaction.toTransitionLabel) => [
                associateWith(reaction)
                applyTransitionOnEdgeStyle()
            ]
        }
    }
    
    def toTransitionLabel(Reaction r) {
        val mode = r.eContainer as Mode
        val transitionReactions = mode.reactions.filter[effects.exists[variable instanceof Mode]].toList
        
        val text = new StringBuilder
        
        text.append(r.triggers.filter(VarRef).map[variable.name].join(", "))
        
        val effects = r.effects.filter[!(variable instanceof Mode)]
        if (!effects.empty) {
            text.append(" / ")
            for(eff : effects) {
                if (eff.container !== null) {
                    text.append(eff.container.name).append(".")
                }
                text.append(eff.variable.name)
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
    
}