package org.icyphy.linguafranca.diagram.synthesis

import com.google.common.collect.HashBasedTable
import com.google.common.collect.HashMultimap
import de.cau.cs.kieler.klighd.KlighdConstants
import de.cau.cs.kieler.klighd.SynthesisOption
import de.cau.cs.kieler.klighd.kgraph.KEdge
import de.cau.cs.kieler.klighd.kgraph.KLabel
import de.cau.cs.kieler.klighd.kgraph.KNode
import de.cau.cs.kieler.klighd.kgraph.KPort
import de.cau.cs.kieler.klighd.krendering.Colors
import de.cau.cs.kieler.klighd.krendering.KPolyline
import de.cau.cs.kieler.klighd.krendering.KRenderingFactory
import de.cau.cs.kieler.klighd.krendering.LineStyle
import de.cau.cs.kieler.klighd.krendering.extensions.KColorExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KEdgeExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KLabelExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KNodeExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KPolylineExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KPortExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceX
import de.cau.cs.kieler.klighd.krendering.extensions.PositionReferenceY
import de.cau.cs.kieler.klighd.labels.decoration.LabelDecorationConfigurator
import de.cau.cs.kieler.klighd.syntheses.AbstractDiagramSynthesis
import de.cau.cs.kieler.klighd.util.KlighdProperties
import java.util.List
import java.util.Map
import javax.inject.Inject
import org.eclipse.elk.alg.layered.options.FixedAlignment
import org.eclipse.elk.alg.layered.options.LayeredOptions
import org.eclipse.elk.alg.layered.p4nodes.bk.EdgeStraighteningStrategy
import org.eclipse.elk.core.options.CoreOptions
import org.eclipse.elk.core.options.Direction
import org.eclipse.elk.core.options.PortConstraints
import org.eclipse.elk.core.options.PortSide
import org.eclipse.elk.core.options.SizeConstraint
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.Connection
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Model
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.TimeUnit
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.VarRef

import static extension de.cau.cs.kieler.klighd.syntheses.DiagramSyntheses.*

class LinguaFrancaSynthesis extends AbstractDiagramSynthesis<Model> {

	@Inject extension KNodeExtensions
	@Inject extension KEdgeExtensions
	@Inject extension KPortExtensions
	@Inject extension KLabelExtensions
	@Inject extension KRenderingExtensions
	@Inject extension KContainerRenderingExtensions
	@Inject extension KPolylineExtensions
	@Inject extension KColorExtensions

	// -------------------------------------------------------------------------
	
	public static val SynthesisOption SHOW_INSTANCE_NAMES = SynthesisOption.createCheckOption("Instance Names", false)
	
	override getDisplayedSynthesisOptions() {
		return #[
			SHOW_INSTANCE_NAMES
		]
	}
	
	// -------------------------------------------------------------------------
	
	override KNode transform(Model model) {
		val rootNode = createNode()

		rootNode.addLayoutParam(CoreOptions.ALGORITHM, LayeredOptions.ALGORITHM_ID)
		rootNode.addLayoutParam(CoreOptions.DIRECTION, Direction.RIGHT)
		rootNode.addLayoutParam(LayeredOptions.NODE_PLACEMENT_BK_FIXED_ALIGNMENT, FixedAlignment.BALANCED)
		rootNode.addLayoutParam(LayeredOptions.NODE_PLACEMENT_BK_EDGE_STRAIGHTENING, EdgeStraighteningStrategy.IMPROVE_STRAIGHTNESS)
		rootNode.addLayoutParam(LayeredOptions.SPACING_EDGE_NODE, LayeredOptions.SPACING_NODE_NODE.^default * 1.1f)
		rootNode.addLayoutParam(LayeredOptions.SPACING_EDGE_NODE_BETWEEN_LAYERS, LayeredOptions.SPACING_NODE_NODE.^default * 1.1f)

		try {
			// Find main
			val main = model.reactors.findFirst[main]
			if (main !== null && main.hasContent) {
				rootNode.children += main.transformReactorNetwork(emptyMap, emptyMap)
			} else {
				val messageNode = createNode()
				messageNode.addErrorMessage("No Main", "Cannot find main reactor with content.")
				rootNode.children += messageNode
			}
		} catch (Exception e) {
			e.printStackTrace
			
			val messageNode = createNode()
			messageNode.addErrorMessage("Error in Diagram Synthesis", e.class.simpleName + " occurred. Could not create diagram.")
			rootNode.children += messageNode
		}

		return rootNode
	}
	
	private def addErrorMessage(KNode node, String title, String message) {
		node.addRectangle() => [
            invisible = true
            addRoundedRectangle(7, 7) => [
                setGridPlacement(1)
                lineWidth = 2
                // title
                if (title !== null) {
	                addText(title) => [
	                    fontSize = 12
	                    setFontBold = true
	                    foreground = Colors.RED
	                    setGridPlacementData().from(LEFT, 8, 0, TOP, 8, 0).to(RIGHT, 8, 0, BOTTOM, 4, 0)
	                    suppressSelectability()
	                ]
                }
                // message
                if (message !== null) {
	                addText(message) => [
                        if (title !== null) {
                            setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 4, 0);
                        } else {
                            setGridPlacementData().from(LEFT, 8, 0, TOP, 8, 0).to(RIGHT, 8, 0, BOTTOM, 8, 0);
                        }
	                ]
	            }
            ]
		]
	}

	private def List<KNode> transformReactorNetwork(Reactor reactor, Map<Input, KPort> parentInputPorts, Map<Output, KPort> parentOutputPorts) {
		val nodes = <KNode>newArrayList
		val inputPorts = HashBasedTable.<Instantiation, Input, KPort>create
		val outputPorts = HashBasedTable.<Instantiation, Output, KPort>create
		val reactionNodes = <Reaction, KNode>newHashMap
		val timerNodes = <Timer, KNode>newHashMap
		
		// Transform instances
		for (instance : reactor.instantiations) {
			val node = createNode()
			nodes += node

			val reactorClass = instance.reactorClass

			node.associateWith(reactorClass)
			node.setLayoutOption(CoreOptions.NODE_SIZE_CONSTRAINTS, SizeConstraint.minimumSizeWithPorts)
			node.setLayoutOption(CoreOptions.PORT_CONSTRAINTS, PortConstraints.FIXED_ORDER)
			node.setLayoutOption(KlighdProperties.EXPAND, false)

			// Expanded Rectangle
			node.addReactorFigure(reactorClass, instance.name) => [
				setProperty(KlighdProperties.EXPANDED_RENDERING, true)

				// Collapse button
				addText("[Hide]") => [
					foreground = Colors.BLUE
					fontSize = 8
					addSingleClickAction(KlighdConstants.ACTION_COLLAPSE_EXPAND)
					addDoubleClickAction(KlighdConstants.ACTION_COLLAPSE_EXPAND)
					setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 0, 0)
				]

				addChildArea()
			]

			// Collapse Rectangle
			node.addReactorFigure(reactorClass, instance.name) => [
				setProperty(KlighdProperties.COLLAPSED_RENDERING, true)

				// Expand button
				if (reactorClass.hasContent) {
					it.addText("[Details]") => [
						foreground = Colors.BLUE
						fontSize = 9
						addSingleClickAction(KlighdConstants.ACTION_COLLAPSE_EXPAND)
						addDoubleClickAction(KlighdConstants.ACTION_COLLAPSE_EXPAND)
						setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 8, 0)
					]
				}
			]

			// Create ports
			for (input : reactorClass.inputs.reverseView) {
				inputPorts.put(instance, input, node.addIOPort(input.name, true))
			}
			for (output : reactorClass.outputs) {
				outputPorts.put(instance, output, node.addIOPort(output.name, false))
			}

			// Add content
			if (reactorClass.hasContent) {
				node.children += reactorClass.transformReactorNetwork(inputPorts.row(instance), outputPorts.row(instance))
			}
		}
		
		// Create timers
		for (Timer timer : reactor.timers?:emptyList) {
			val node = createNode().associateWith(timer)
			nodes += node
			timerNodes.put(timer, node)
			
			node.addTimerFigure(timer)
		}

		// Create reactions
		val actionDestinations = HashMultimap.<Action, Reaction>create
		val actionSource = <Action, Reaction>newHashMap
		for (reaction : reactor.reactions) {
			val node = createNode().associateWith(reaction)
			nodes += node
			reactionNodes.put(reaction, node)
			
			node.setLayoutOption(CoreOptions.PORT_CONSTRAINTS, PortConstraints.FREE)
			
			node.addReactionFigure(reaction)

			// connect input
			for (VarRef trigger : reaction.triggers?:emptyList) {
				if (trigger.variable instanceof Action) {
					actionDestinations.put(trigger.variable as Action, reaction)		
				} else {
					val src = if (parentInputPorts.containsKey(trigger.variable)) {
						parentInputPorts.get(trigger.variable)
					} else if (timerNodes.containsKey(trigger.variable)) {
						timerNodes.get(trigger.variable)
					}
					if (src !== null) {
						createDependencyEdge().connect(src, node)
					}
				}
			}
			
			// connect outputs
			for (VarRef effect : reaction.effects?:emptyList) {
				if (effect.variable instanceof Action) {
					actionSource.put(effect.variable as Action, reaction)
				} else {
					val dst = if (effect.variable instanceof Output) {
						parentOutputPorts.get(effect.variable)
					} else {
						inputPorts.get(effect.container, effect.variable)
					}
					if (dst !== null) {
						createDependencyEdge().connect(node, dst)
					}
				}
			}
		}
		
		// Connect actions
		for (Action action : actionSource.keySet) {
			val sourceNode = reactionNodes.get(actionSource.get(action))
			for (target : actionDestinations.get(action)) {
				val targetNode  = reactionNodes.get(target)
				createDelayEdge(action).connect(sourceNode, targetNode)
			}
		}

		// Transform connections
		for (Connection connection : reactor.connections?:emptyList) {
			val source = if (connection.leftPort.container !== null) {
				outputPorts.get(connection.leftPort.container, connection.leftPort.variable)
			} else if (parentInputPorts.containsKey(connection.leftPort.variable)) {
				parentInputPorts.get(connection.leftPort.variable)
			}
			val target = if (connection.rightPort.container !== null) {
				inputPorts.get(connection.rightPort.container, connection.rightPort.variable)
			} else if (parentOutputPorts.containsKey(connection.rightPort.variable)) {
				parentOutputPorts.get(connection.rightPort.variable)
			}
			val edge = createDependencyEdge.associateWith(connection)
			edge.connect(source, target)
		}

		return nodes
	}
	
	private def createDelayEdge(Action action) {
		return createEdge => [
			addPolyline() => [
				//.addHeadArrowDecorator() // added by connect
				lineStyle = LineStyle.DASH
			]
			if (action.delay?.unit == TimeUnit.NONE) {
				addCenterEdgeLabel(action.delay.value).applyOnEdgeStyle()
			}
		]
	}
	
	
	private def createDependencyEdge() {
		return createEdge => [
			addPolyline()//.addHeadArrowDecorator() // added by connect
		]
	}
	
	private def dispatch connect(KEdge edge, KNode src, KNode dst) {
		(edge.KContainerRendering as KPolyline).addHeadArrowDecorator()
		edge.source = src
		edge.target = dst
	}
	private def dispatch connect(KEdge edge, KNode src, KPort dst) {
		edge.source = src
		edge.targetPort = dst
		edge.target = dst?.node
	}
	private def dispatch connect(KEdge edge, KPort src, KNode dst) {
		(edge.KContainerRendering as KPolyline).addHeadArrowDecorator()
		edge.sourcePort = src
		edge.source = src?.node
		edge.target = dst
	}
	private def dispatch connect(KEdge edge, KPort src, KPort dst) {
		edge.sourcePort = src
		edge.source = src?.node
		edge.targetPort = dst
		edge.target = dst?.node
	}
	
	/**
	 * Translate an input/output into a port.
	 */
	private def addIOPort(KNode node, String label, boolean input) {
		val port = createPort
		node.ports += port
		
		port.setPortSize(6, 6)
		
		if (input) {
			port.addLayoutParam(CoreOptions.PORT_SIDE, PortSide.WEST)
			port.addLayoutParam(CoreOptions::PORT_BORDER_OFFSET, -3.0)
		} else {
			port.addLayoutParam(CoreOptions.PORT_SIDE, PortSide.EAST)
			port.addLayoutParam(CoreOptions::PORT_BORDER_OFFSET, -3.0)
		}
		
		port.addPolygon() => [
			lineWidth = 1
			background = Colors.BLACK
			points += #[
				createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.TOP, 0 , 0),
				createKPosition(PositionReferenceX.RIGHT, 0, 0, PositionReferenceY.TOP, 0 , 0.5f),
				createKPosition(PositionReferenceX.LEFT, 0, 0, PositionReferenceY.BOTTOM, 0 , 0)
			]
		]
		
		port.addOutsidePortLabel(label, 8)

		return port
	}

	/**
	 * Creates the visual representation of a reactor node
	 */
	private def addReactorFigure(KNode node, Reactor reactor, String instanceName) {
		val figure = node.addRoundedRectangle(8, 8, 1)
		figure.setGridPlacement(1)
		figure.lineWidth = 1
		figure.foreground = Colors.GRAY
		figure.background = Colors.GRAY_95

		// minimal node size is necessary if no text will be added
		node.setMinimalNodeSize(2 * figure.cornerWidth, 2 * figure.cornerHeight)

		val showInstanceName = SHOW_INSTANCE_NAMES.booleanValue && !instanceName.nullOrEmpty
		
		figure.addText(reactor.name) => [
			setGridPlacementData().from(LEFT, 8, 0, TOP, 8f, 0).to(RIGHT, 8, 0, BOTTOM, showInstanceName ? 0 : (reactor.hasContent ? 4 : 8), 0)
			suppressSelectability
		]
		
		if (showInstanceName) {
			figure.addText(instanceName) => [
				fontItalic = true
				setGridPlacementData().from(LEFT, 8, 0, TOP, 2, 0).to(RIGHT, 8, 0, BOTTOM, reactor.hasContent ? 4 : 8, 0)
				suppressSelectability
			]
		}

		return figure
	}
	
	/**
	 * Creates the visual representation of a reaction node
	 */
	private def addReactionFigure(KNode node, Reaction reaction) {
		node.setMinimalNodeSize(45, 15)

		val figure = node.addRectangle()
		figure.lineWidth = 1
		figure.foreground = Colors.GRAY_45
		figure.background = Colors.GRAY_65

		return figure
	}
	
	/**
	 * Creates the visual representation of a timer node
	 */
	private def addTimerFigure(KNode node, Timer timer) {
		node.setMinimalNodeSize(40, 40)
		
		val figure = node.addEllipse
		figure.lineWidth = 1
		figure.background = Colors.GRAY_95
		
		figure.addPolyline(1,
			#[
				createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 0 , 0.1f),
				createKPosition(PositionReferenceX.LEFT, 0, 0.5f, PositionReferenceY.TOP, 0 , 0.5f),
				createKPosition(PositionReferenceX.LEFT, 0, 0.7f, PositionReferenceY.TOP, 0 , 0.7f)
			]
		)
		
		if (timer.timing !== null) {
			val timing = newArrayList
			if (timer.timing.offset !== null) {
				timing += timer.timing.offset.time + timer.timing.offset.unit.name()
			}
			if (timer.timing.period !== null) {
				timing += timer.timing.period.time + timer.timing.period.unit.name()
			}
			if (!timing.empty) {
				node.addOutsideBottomCenteredNodeLabel(timing.join("(", ", ", ")")[it])
			}
		}

		return figure
	}
	
	private def hasContent(Reactor reactor) {
		return !reactor.reactions.empty || !reactor.instantiations.empty
	}
	
	static var LabelDecorationConfigurator _inlineLabelConfigurator; // ONLY for use in applyOnEdgeStyle
	private def applyOnEdgeStyle(KLabel label) {
		if (_inlineLabelConfigurator === null) {
	        _inlineLabelConfigurator = LabelDecorationConfigurator.create
	        	.withInlineLabels(true)
	            .withLabelTextRenderingProvider([ container, klabel | 
	            	val kText = KRenderingFactory.eINSTANCE.createKText()
	            	kText.fontSize = 9
        			container.children += kText
        			kText
	            ])
		}
		
		_inlineLabelConfigurator.applyTo(label)
	}

}
