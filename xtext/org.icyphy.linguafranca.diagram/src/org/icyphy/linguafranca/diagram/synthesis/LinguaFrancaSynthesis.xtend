package org.icyphy.linguafranca.diagram.synthesis

import com.google.common.collect.HashBasedTable
import com.google.common.collect.HashMultimap
import de.cau.cs.kieler.klighd.DisplayedActionData
import de.cau.cs.kieler.klighd.SynthesisOption
import de.cau.cs.kieler.klighd.kgraph.KEdge
import de.cau.cs.kieler.klighd.kgraph.KNode
import de.cau.cs.kieler.klighd.kgraph.KPort
import de.cau.cs.kieler.klighd.krendering.LineStyle
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared
import de.cau.cs.kieler.klighd.krendering.extensions.KColorExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KEdgeExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KLabelExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KNodeExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KPolylineExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KPortExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions
import de.cau.cs.kieler.klighd.syntheses.AbstractDiagramSynthesis
import de.cau.cs.kieler.klighd.util.KlighdProperties
import java.util.EnumSet
import java.util.List
import java.util.Map
import javax.inject.Inject
import org.eclipse.elk.alg.layered.options.FixedAlignment
import org.eclipse.elk.alg.layered.options.LayerConstraint
import org.eclipse.elk.alg.layered.options.LayeredOptions
import org.eclipse.elk.alg.layered.p4nodes.bk.EdgeStraighteningStrategy
import org.eclipse.elk.core.options.CoreOptions
import org.eclipse.elk.core.options.Direction
import org.eclipse.elk.core.options.PortConstraints
import org.eclipse.elk.core.options.PortSide
import org.eclipse.elk.core.options.SizeConstraint
import org.eclipse.elk.graph.properties.Property
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.ActionOrigin
import org.icyphy.linguaFranca.Connection
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Model
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Reaction
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguaFranca.Timer
import org.icyphy.linguaFranca.TriggerRef
import org.icyphy.linguaFranca.VarRef
import org.icyphy.linguaFranca.Variable
import org.icyphy.linguafranca.diagram.synthesis.action.CollapseAllReactorsAction
import org.icyphy.linguafranca.diagram.synthesis.action.ExpandAllReactorsAction
import org.icyphy.linguafranca.diagram.synthesis.styles.LinguaFrancaShapeExtensions
import org.icyphy.linguafranca.diagram.synthesis.styles.LinguaFrancaStyleExtensions

import static extension org.icyphy.linguafranca.diagram.synthesis.action.MemorizingExpandCollapseAction.*

@ViewSynthesisShared
class LinguaFrancaSynthesis extends AbstractDiagramSynthesis<Model> {

	@Inject extension KNodeExtensions
	@Inject extension KEdgeExtensions
	@Inject extension KPortExtensions
	@Inject extension KLabelExtensions
	@Inject extension KRenderingExtensions
	@Inject extension KContainerRenderingExtensions
	@Inject extension KPolylineExtensions
	@Inject extension KColorExtensions
	@Inject extension LinguaFrancaStyleExtensions
	@Inject extension LinguaFrancaShapeExtensions
	@Inject extension LinguaFrancaSynthesisUtilityExtensions
	
	// -------------------------------------------------------------------------
	
	public static val REACTOR_INSTANCE = new Property<Instantiation>("org.icyphy.linguafranca.diagram.synthesis.reactor.instantiation")
	static val ACTION_NODES = true // just for toggling internal modes

	// -------------------------------------------------------------------------
	
	/** Synthesis options */
	public static val SynthesisOption SHOW_MAIN_REACTOR = SynthesisOption.createCheckOption("Main Reactor Frame", true)
	public static val SynthesisOption SHOW_INSTANCE_NAMES = SynthesisOption.createCheckOption("Instance Names", false)
	public static val SynthesisOption REACTIONS_USE_HYPEREDGES = SynthesisOption.createCheckOption("Bundled Dependencies", false)
	public static val SynthesisOption SHOW_REACTION_CODE = SynthesisOption.createCheckOption("Reaction Code", false)
	
    /** Synthesis actions */
    public static val DisplayedActionData COLLAPSE_ALL = DisplayedActionData.create(CollapseAllReactorsAction.ID, "Hide all Details")
    public static val DisplayedActionData EXPAND_ALL = DisplayedActionData.create(ExpandAllReactorsAction.ID, "Show all Details")
	
	override getDisplayedSynthesisOptions() {
		return #[
			SHOW_MAIN_REACTOR,
			SHOW_INSTANCE_NAMES,
			REACTIONS_USE_HYPEREDGES,
			SHOW_REACTION_CODE,
			MEMORIZE_EXPANSION_STATES
		]
	}
	
    override getDisplayedActions() {
        return #[COLLAPSE_ALL, EXPAND_ALL]
    }
	
	// -------------------------------------------------------------------------
	
	override KNode transform(Model model) {
		val rootNode = createNode()
		var mainNode = rootNode

		try {
			// Find main
			val main = model.reactors.findFirst[main]
			if (main !== null && main.hasContent) {
				val nodes = main.transformReactorNetwork(emptyMap, emptyMap)
				if (SHOW_MAIN_REACTOR.booleanValue) {
					mainNode = createNode(main)
					mainNode.associateWith(main)
					mainNode.ID = "main"
					mainNode.addMainReactorFigure(main) => [
						associateWith(main)
						addChildArea()
					]
					mainNode.children += nodes
					rootNode.children += mainNode
					
					// only for main reactor node
					mainNode.setLayoutOption(CoreOptions::NODE_SIZE_CONSTRAINTS, EnumSet.of(SizeConstraint.MINIMUM_SIZE))
				} else {
					rootNode.children += nodes
				}
				
				mainNode.addLayoutParam(CoreOptions.ALGORITHM, LayeredOptions.ALGORITHM_ID)
				mainNode.addLayoutParam(CoreOptions.DIRECTION, Direction.RIGHT)
				mainNode.addLayoutParam(LayeredOptions.NODE_PLACEMENT_BK_FIXED_ALIGNMENT, FixedAlignment.BALANCED)
				mainNode.addLayoutParam(LayeredOptions.NODE_PLACEMENT_BK_EDGE_STRAIGHTENING, EdgeStraighteningStrategy.IMPROVE_STRAIGHTNESS)
				mainNode.addLayoutParam(LayeredOptions.SPACING_EDGE_NODE, LayeredOptions.SPACING_NODE_NODE.^default * 1.1f)
				mainNode.addLayoutParam(LayeredOptions.SPACING_EDGE_NODE_BETWEEN_LAYERS, LayeredOptions.SPACING_NODE_NODE.^default * 1.1f)
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

	private def List<KNode> transformReactorNetwork(Reactor reactor, Map<Input, KPort> parentInputPorts, Map<Output, KPort> parentOutputPorts) {
		val nodes = <KNode>newArrayList
		val inputPorts = HashBasedTable.<Instantiation, Input, KPort>create
		val outputPorts = HashBasedTable.<Instantiation, Output, KPort>create
		val reactionNodes = <Reaction, KNode>newHashMap
		val actionDestinations = HashMultimap.<Action, KPort>create
		val actionSource = <Action, KPort>newHashMap
		val timerNodes = <Timer, KNode>newHashMap
		val startupNode = createNode
		var startupUsed = false
		val shutdownNode = createNode
		var shutdownUsed = false
		
		// Transform instances
		for (instance : reactor.instantiations) {
			val node = createNode()
			nodes += node
			val reactorClass = instance.reactorClass

			node.associateWith(reactorClass)
			node.setLayoutOption(CoreOptions.NODE_SIZE_CONSTRAINTS, SizeConstraint.minimumSizeWithPorts)
			node.setLayoutOption(CoreOptions.PORT_CONSTRAINTS, PortConstraints.FIXED_ORDER)
			node.setLayoutOption(KlighdProperties.EXPAND, instance.getExpansionState?:false)
			node.setProperty(REACTOR_INSTANCE, instance) // save to distinguish nodes associated with the same reactor

			// Expanded Rectangle
			node.addReactorFigure(reactorClass, instance.name) => [
				setProperty(KlighdProperties.EXPANDED_RENDERING, true)
				addDoubleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)
				boldLineSelectionStyle

				// Collapse button
				addTextButton("[Hide]") => [
					setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 0, 0)
					addSingleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)
					addDoubleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)
				]

				addChildArea()
			]

			// Collapse Rectangle
			node.addReactorFigure(reactorClass, instance.name) => [
				setProperty(KlighdProperties.COLLAPSED_RENDERING, true)
				addDoubleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)
				boldLineSelectionStyle

				// Expand button
				if (reactorClass.hasContent) {
					addTextButton("[Details]") => [
						setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 8, 0)
						addSingleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)
						addDoubleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)
					]
				}
			]

			// Create ports
			for (input : reactorClass.inputs.reverseView) {
				inputPorts.put(instance, input, node.addIOPort(input, true))
			}
			for (output : reactorClass.outputs) {
				outputPorts.put(instance, output, node.addIOPort(output, false))
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
		for (reaction : reactor.reactions.reverseView) {
			val node = createNode().associateWith(reaction)
			nodes += node
			reactionNodes.put(reaction, node)
			
			node.setLayoutOption(CoreOptions.PORT_CONSTRAINTS, PortConstraints.FIXED_SIDE)
			node.addReactionFigure(reaction)
		
			// connect input
			var KPort port
			for (TriggerRef trigger : reaction.triggers?:emptyList) {
				port = if (REACTIONS_USE_HYPEREDGES.booleanValue && port !== null) {
					port
				} else {
					node.addInvisiblePort() => [
						addLayoutParam(CoreOptions.PORT_SIDE, PortSide.WEST)
						if (REACTIONS_USE_HYPEREDGES.booleanValue || ((reaction.triggers?:emptyList).size + (reaction.sources?:emptyList).size) == 1) {
							addLayoutParam(CoreOptions::PORT_BORDER_OFFSET, -LinguaFrancaShapeExtensions::REACTION_POINTINESS as double) // requires offset due to shape
						} else {
							addLayoutParam(CoreOptions::PORT_BORDER_OFFSET, (-LinguaFrancaShapeExtensions::REACTION_POINTINESS as double) * 0.33) // requires offset due to shape
						}
					]
				}
				if (trigger instanceof VarRef) {
					if (trigger.variable instanceof Action) {
						actionDestinations.put(trigger.variable as Action, port)
					} else {
						val src = if (trigger.container !== null) {
							outputPorts.get(trigger.container, trigger.variable)
						} else if (parentInputPorts.containsKey(trigger.variable)) {
							parentInputPorts.get(trigger.variable)
						} else if (timerNodes.containsKey(trigger.variable)) {
							timerNodes.get(trigger.variable)
						}
						if (src !== null) {
							createDependencyEdge(trigger).connect(src, port)
						}
					}
				} else if (trigger.startup) {
					createDependencyEdge(trigger).connect(startupNode, port)
					startupUsed = true
				} else if (trigger.shutdown) {
					createDelayEdge(trigger).connect(shutdownNode, port)
					shutdownUsed = true
				}
			}
			
			// connect dependencies
			//port = null
			for (VarRef dep : reaction.sources?:emptyList) {
				port = if (REACTIONS_USE_HYPEREDGES.booleanValue && port !== null) {
					port
				} else {
					node.addInvisiblePort() => [
						//addLayoutParam(CoreOptions.PORT_SIDE, PortSide.NORTH)
						addLayoutParam(CoreOptions.PORT_SIDE, PortSide.WEST)
						if (REACTIONS_USE_HYPEREDGES.booleanValue || ((reaction.triggers?:emptyList).size + (reaction.sources?:emptyList).size) == 1) {
							addLayoutParam(CoreOptions::PORT_BORDER_OFFSET, -LinguaFrancaShapeExtensions::REACTION_POINTINESS as double) // requires offset due to shape
						} else {
							addLayoutParam(CoreOptions::PORT_BORDER_OFFSET, (-LinguaFrancaShapeExtensions::REACTION_POINTINESS as double) * 0.33) // requires offset due to shape
						}
					]
				}
				if (dep.variable instanceof Action) { // TODO I think this case is forbidden
					actionDestinations.put(dep.variable as Action, port)
				} else {
					val src = if (dep.container !== null) {
						outputPorts.get(dep.container, dep.variable)
					} else if (parentInputPorts.containsKey(dep.variable)) {
						parentInputPorts.get(dep.variable)
					} else if (timerNodes.containsKey(dep.variable)) { // TODO I think this is forbidden
						timerNodes.get(dep.variable)
					}
					if (src !== null) {
						createDependencyEdge(dep).connect(src, port)
					}
				}
			}
			
			// connect outputs
			port = null
			for (VarRef effect : reaction.effects?:emptyList) {
				port = if (REACTIONS_USE_HYPEREDGES.booleanValue && port !== null) {
					port
				} else {
					node.addInvisiblePort() => [
						addLayoutParam(CoreOptions.PORT_SIDE, PortSide.EAST)
					]
				}
				if (effect.variable instanceof Action) {
					actionSource.put(effect.variable as Action, port)
				} else {
					val dst = if (effect.variable instanceof Output) {
						parentOutputPorts.get(effect.variable)
					} else {
						inputPorts.get(effect.container, effect.variable)
					}
					if (dst !== null) {
						createDependencyEdge(effect).connect(port, dst)
					}
				}
			}
		}
		
		// Connect actions
		for (Action action : actionSource.keySet) {
			val sourcePort = actionSource.get(action)
			if (ACTION_NODES) {
				val node = createNode().associateWith(action)
				nodes += node
				node.setLayoutOption(CoreOptions.PORT_CONSTRAINTS, PortConstraints.FIXED_SIDE)
				val ports = node.addActionFigureAndPorts(action.origin === ActionOrigin.PHYSICAL ? "P" : "L")
				if (action.delay !== null) {
					node.addOutsideBottomCenteredNodeLabel(action.delay.toText, 7)
				}
				
				createDelayEdge(action).connect(sourcePort, ports.key)
				for (target : actionDestinations.get(action)) {
					createDelayEdge(action).connect(ports.value, target)
				}
			} else {
				for (target : actionDestinations.get(action)) {
					createDelayEdge(action).connect(sourcePort, target)
				}
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
			val edge = createIODependencyEdge(connection).associateWith(connection)
			edge.connect(source, target)
		}
		
		// Add startup/shutdown
		if (startupUsed) {
			startupNode.addStartupFigure
			nodes.add(0, startupNode)
			startupNode.setLayoutOption(LayeredOptions.LAYERING_LAYER_CONSTRAINT, LayerConstraint.FIRST)
			if (REACTIONS_USE_HYPEREDGES.booleanValue) { // connect all edges to one port
				val port = startupNode.addInvisiblePort
				startupNode.outgoingEdges.forEach[sourcePort = port]
			}
		}
		if (shutdownUsed) {
			shutdownNode.addShutdownFigure
			nodes.add(0, shutdownNode)
			if (REACTIONS_USE_HYPEREDGES.booleanValue) { // connect all edges to one port
				val port = shutdownNode.addInvisiblePort
				shutdownNode.outgoingEdges.forEach[sourcePort = port]
			}
		}
		
		// Postprocess timer nodes
		if (REACTIONS_USE_HYPEREDGES.booleanValue) { // connect all edges to one port
			for (timerNode : timerNodes.values) {
				val port = timerNode.addInvisiblePort
				timerNode.outgoingEdges.forEach[sourcePort = port]
			}
		}
		
		return nodes
	}
	
	private def createDelayEdge(Object associate) {
		return createEdge => [
			associateWith(associate)
			val line = addPolyline() => [
				lineStyle = LineStyle.DASH
				boldLineSelectionStyle()
			]
			if (associate instanceof Action && !ACTION_NODES) {
				val action = associate as Action
				line.addActionDecorator(action.origin === ActionOrigin.PHYSICAL ? "P" : "L")
				if (action.delay !== null) {
					addCenterEdgeLabel(action.delay.toText)//.applyOnEdgeStyle()
				}
			}
		]
	}
	
	private def createIODependencyEdge(Object associate) {
		return createEdge => [
			if (associate !== null) {
				associateWith(associate)
			}
			addPolyline() => [
				boldLineSelectionStyle()
			]
		]
	}
	
	private def createDependencyEdge(Object associate) {
		return createEdge => [
			if (associate !== null) {
				associateWith(associate)
			}
			addPolyline() => [
				lineStyle = LineStyle.DASH
				boldLineSelectionStyle()
			]
		]
	}
	
	private def dispatch KEdge connect(KEdge edge, KNode src, KNode dst) {
		edge.source = src
		edge.target = dst
		
		return edge
	}
	private def dispatch KEdge connect(KEdge edge, KNode src, KPort dst) {
		edge.source = src
		edge.targetPort = dst
		edge.target = dst?.node
		
		return edge
	}
	private def dispatch KEdge connect(KEdge edge, KPort src, KNode dst) {
		edge.sourcePort = src
		edge.source = src?.node
		edge.target = dst
		
		return edge
	}
	private def dispatch KEdge connect(KEdge edge, KPort src, KPort dst) {
		edge.sourcePort = src
		edge.source = src?.node
		edge.targetPort = dst
		edge.target = dst?.node
		
		return edge
	}
	
	/**
	 * Translate an input/output into a port.
	 */
	private def addIOPort(KNode node, Variable variable, boolean input) {
		val port = createPort
		node.ports += port
		
		port.associateWith(variable)
		port.setPortSize(6, 6)
		
		if (input) {
			port.addLayoutParam(CoreOptions.PORT_SIDE, PortSide.WEST)
			port.addLayoutParam(CoreOptions::PORT_BORDER_OFFSET, -3.0)
		} else {
			port.addLayoutParam(CoreOptions.PORT_SIDE, PortSide.EAST)
			port.addLayoutParam(CoreOptions::PORT_BORDER_OFFSET, -3.0)
		}
		
		port.addTrianglePort()
		port.addOutsidePortLabel(variable.name, 8).associateWith(variable)

		return port
	}

	private def KPort addInvisiblePort(KNode node) {
		val port = createPort
		node.ports += port
		
		port.setSize(0, 0) // invisible

		return port
	}

}
