package org.icyphy.linguafranca.diagram.synthesis

import com.google.common.collect.HashBasedTable
import com.google.common.collect.Table
import de.cau.cs.kieler.klighd.KlighdConstants
import de.cau.cs.kieler.klighd.kgraph.KNode
import de.cau.cs.kieler.klighd.kgraph.KPort
import de.cau.cs.kieler.klighd.kgraph.util.KGraphUtil
import de.cau.cs.kieler.klighd.krendering.Colors
import de.cau.cs.kieler.klighd.krendering.extensions.KColorExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KEdgeExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KLabelExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KNodeExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KPolylineExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KPortExtensions
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions
import de.cau.cs.kieler.klighd.microlayout.PlacementUtil
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
import org.eclipse.elk.core.options.PortSide
import org.eclipse.elk.core.options.SizeConstraint
import org.icyphy.linguaFranca.Model
import org.icyphy.linguaFranca.Reactor

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
	override KNode transform(Model model) {
		val rootNode = createNode()

		rootNode.addLayoutParam(CoreOptions.ALGORITHM, LayeredOptions.ALGORITHM_ID)
		rootNode.addLayoutParam(CoreOptions.DIRECTION, Direction.RIGHT)
		rootNode.setLayoutOption(LayeredOptions.NODE_PLACEMENT_BK_FIXED_ALIGNMENT, FixedAlignment.BALANCED)
		rootNode.setLayoutOption(LayeredOptions.NODE_PLACEMENT_BK_EDGE_STRAIGHTENING, EdgeStraighteningStrategy.IMPROVE_STRAIGHTNESS)
		rootNode.setLayoutOption(LayeredOptions.SPACING_EDGE_NODE, LayeredOptions.SPACING_NODE_NODE.^default * 1.1f)
		rootNode.setLayoutOption(LayeredOptions.SPACING_EDGE_NODE_BETWEEN_LAYERS, LayeredOptions.SPACING_NODE_NODE.^default * 1.1f)

		model.prepareStructureAccess() // TODO usually Xtext does this
		// find main
		val main = model.reactors.findFirst["main".equalsIgnoreCase(name)]
		if (main !== null && !main.instances.nullOrEmpty) {
			rootNode.children += main.transformReactorNetwork(emptyMap, emptyMap)
		} else {
			val message = createNode
			message.createReactorFigure
			message.KContainerRendering.addText("No main reactor containing reactor instances")
		}

		return rootNode
	}

	private def List<KNode> transformReactorNetwork(Reactor reactor, Map<String, KPort> parentInputPorts,
		Map<String, KPort> parentOutputPorts) {
		val nodes = newArrayList
		val Table<String, String, KPort> inputPorts = HashBasedTable.create // TODO should be <Reactor, Input, KPort>
		val Table<String, String, KPort> outputPorts = HashBasedTable.create // TODO should be <Reactor, Input, KPort>
		// transform instances
		for (instance : reactor.instances) {
			val node = createNode
			nodes += node

			val name = instance.name
			val clazz = instance.reactorClass.reactor
			val hasContent = !clazz.reactions.empty || !clazz.instances.empty

			node.associateWith(clazz)
			node.setLayoutOption(CoreOptions.NODE_SIZE_CONSTRAINTS, SizeConstraint.free)
			node.setLayoutOption(KlighdProperties.EXPAND, false)

			// Expanded Rectangle
			node.createReactorFigure() => [
				setProperty(KlighdProperties.EXPANDED_RENDERING, true)
				setGridPlacement(1)

				// add name
				addText(instance.reactorClass) => [
					fontSize = 10
					setFontBold(true)
					setGridPlacementData().from(LEFT, 9, 0, TOP, 8f, 0).to(RIGHT, 8, 0, BOTTOM, 4, 0)
					suppressSelectability
				]

				// collapse button
				addText("[Hide]") => [
					foreground = "blue".color
					fontSize = 8
					addSingleClickAction(KlighdConstants.ACTION_COLLAPSE_EXPAND)
					addDoubleClickAction(KlighdConstants.ACTION_COLLAPSE_EXPAND)
					setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 0, 0)
				]

				addChildArea
			]

			// Collapse Rectangle
			node.createReactorFigure() => [
				it.setProperty(KlighdProperties.COLLAPSED_RENDERING, true)
				it.setGridPlacement(1)

				// add name
				addText(instance.reactorClass) => [
					fontSize = 11
					fontBold = true
					setGridPlacementData().from(LEFT, 8, 0, TOP, 8f, 0).to(RIGHT, 8, 0, BOTTOM, hasContent ? 4 : 8, 0)
					suppressSelectability
				]

				// expand button
				if (hasContent) {
					it.addText("[Details]") => [
						foreground = "blue".color
						fontSize = 9
						addSingleClickAction(KlighdConstants.ACTION_COLLAPSE_EXPAND)
						addDoubleClickAction(KlighdConstants.ACTION_COLLAPSE_EXPAND)
						setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 8, 0)
					]
				}
			]

			// Create ports
			for (input : clazz.inputs.reverseView) {
				inputPorts.put(name, input.name, node.createIOPort(input.name, true))
			}
			for (output : clazz.outputs) {
				outputPorts.put(name, output.name, node.createIOPort(output.name, false))
			}

			// Add content
			if (hasContent) {
				node.children += clazz.transformReactorNetwork(inputPorts.row(name), outputPorts.row(name))
			}
		}

		// Create Reactions
		for (reaction : reactor.reactions) {
			val node = createNode
			nodes += node

			// minimal node size is necessary if no text will be added
			node.setMinimalNodeSize(45, 15)

			node.addRectangle => [
				lineWidth = 1
				background = Colors.GRAY_65
			]

			// connect
			for (trigger : reaction.triggers) {
				val port = parentInputPorts.get(trigger)
				if (port !== null) {
					createEdge => [
						associateWith(trigger)
						addPolyline.addHeadArrowDecorator
						source = port.node
						sourcePort = port
						target = node
					]
				}
			}
		}

		// Transform connections
		for (connection : reactor.connections) {
			val left = connection.leftPort.split("\\.") // TODO model should better contain cross references
			val right = connection.rightPort.split("\\.") // TODO model should better contain cross references
			createEdge => [
				associateWith(connection)
				addPolyline.addHeadArrowDecorator
				sourcePort = outputPorts.get(left.get(0), left.get(1))
				source = sourcePort?.node
				targetPort = inputPorts.get(right.get(0), right.get(1))
				target = targetPort?.node
			]
		}

		return nodes
	}

	/**
	 * Translate a structural container feature into a port.
	 */
	private def createIOPort(KNode node, String label, boolean input) {
		val port = KGraphUtil.createInitializedPort
		node.ports += port
		port.setPortSize(portEdgeLength, portEdgeLength)
		port.addLayoutParam(CoreOptions.PORT_SIDE, input ? PortSide.WEST : PortSide.EAST)
		port.setPortPos(node.width - 1, node.nextEPortYPosition)
		port.createLabel => [
			text = label
			val size = PlacementUtil.estimateSize(it)
			width = size.width
			height = size.height
		]

		return port
	}

	/**
	 * Creates the visual representation of a node
	 */
	private def createReactorFigure(KNode node) {
		val figure = node.addRoundedRectangle(8, 8, 1)
		figure.lineWidth = 1
		figure.foreground = Colors.GRAY
		figure.background = Colors.GRAY_95

		// minimal node size is necessary if no text will be added
		node.setMinimalNodeSize(2 * figure.cornerWidth, 2 * figure.cornerHeight)

		return figure
	}

	// -------------------------
	// Model Helpers
	var Map<String, Reactor> reactors

	def void prepareStructureAccess(Model model) {
		reactors = model.reactors.toMap[name]
		// TODO include imported files
	}

	def Reactor reactor(String name) {
		return reactors.get(name)
	}
}
