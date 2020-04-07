package org.icyphy.linguafranca.diagram.synthesis

import com.google.common.collect.HashBasedTable
import com.google.common.collect.HashBiMap
import com.google.common.collect.HashMultimap
import com.google.common.collect.Table
import de.cau.cs.kieler.klighd.DisplayedActionData
import de.cau.cs.kieler.klighd.SynthesisOption
import de.cau.cs.kieler.klighd.internal.util.KlighdInternalProperties
import de.cau.cs.kieler.klighd.kgraph.KEdge
import de.cau.cs.kieler.klighd.kgraph.KNode
import de.cau.cs.kieler.klighd.kgraph.KPort
import de.cau.cs.kieler.klighd.krendering.Colors
import de.cau.cs.kieler.klighd.krendering.HorizontalAlignment
import de.cau.cs.kieler.klighd.krendering.KContainerRendering
import de.cau.cs.kieler.klighd.krendering.KRendering
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
import java.util.Collection
import java.util.EnumSet
import java.util.List
import java.util.Map
import javax.inject.Inject
import org.eclipse.elk.alg.layered.options.FixedAlignment
import org.eclipse.elk.alg.layered.options.LayerConstraint
import org.eclipse.elk.alg.layered.options.LayeredOptions
import org.eclipse.elk.alg.layered.p4nodes.bk.EdgeStraighteningStrategy
import org.eclipse.elk.core.math.ElkPadding
import org.eclipse.elk.core.math.KVector
import org.eclipse.elk.core.options.BoxLayouterOptions
import org.eclipse.elk.core.options.CoreOptions
import org.eclipse.elk.core.options.Direction
import org.eclipse.elk.core.options.PortConstraints
import org.eclipse.elk.core.options.PortSide
import org.eclipse.elk.core.options.SizeConstraint
import org.eclipse.elk.graph.properties.Property
import org.icyphy.ASTUtils
import org.icyphy.AnnotatedDependencyGraph
import org.icyphy.AnnotatedNode
import org.icyphy.linguaFranca.Action
import org.icyphy.linguaFranca.ActionOrigin
import org.icyphy.linguaFranca.Connection
import org.icyphy.linguaFranca.Input
import org.icyphy.linguaFranca.Instantiation
import org.icyphy.linguaFranca.Model
import org.icyphy.linguaFranca.Output
import org.icyphy.linguaFranca.Parameter
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

import static extension de.cau.cs.kieler.klighd.kgraph.util.KGraphIterators.*
import static extension java.lang.String.format
import static extension org.eclipse.emf.ecore.util.EcoreUtil.*
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

	// -- INTERNAL --
	public static val REACTOR_INSTANCE = new Property<Instantiation>("org.icyphy.linguafranca.diagram.synthesis.reactor.instantiation")
	public static val RECURSIVE_INSTANTIATION = new Property<Boolean>("org.icyphy.linguafranca.diagram.synthesis.recursive.instantiation", false)
	public static val DEPENDENCY_CYCLE = new Property<Boolean>("org.icyphy.linguafranca.diagram.synthesis.dependency.cycle", false)
	private static val DEBUG_CYCLE_DETECTION = false

	// -- STYLE --	
	public static val ALTERNATIVE_DASH_PATTERN = #[3.0f]
	
	// -- TEXT --
	public static val TEXT_ERROR_RECURSIVE = "Recursive reactor instantiation!"
	public static val TEXT_ERROR_CONTAINS_RECURSION = "Reactor contains recursive instantiation!"
	public static val TEXT_ERROR_CONTAINS_CYCLE = "Reactor contains cyclic dependencies!"
	public static val TEXT_ERROR_CYCLE_DETECTION = "Dependency cycle detection failed.\nCould not detect dependency cycles due to unexpected graph structure."
	public static val TEXT_ERROR_CYCLE_BTN_SHOW = "Show Cycle (Reactor Details)"
	public static val TEXT_NO_MAIN_REACTOR = "No Main Reactor"
	public static val TEXT_REACTOR_NULL = "Reactor is null"
	public static val TEXT_HIDE_ACTION = "[Hide]"
	public static val TEXT_SHOW_ACTION = "[Details]"
	
	// -------------------------------------------------------------------------
	
	/** Synthesis category */
	public static val SynthesisOption APPEARANCE = SynthesisOption.createCategory("Appearance", true)
	public static val SynthesisOption EXPERIMENTAL = SynthesisOption.createCategory("Experimental", true)
	
	/** Synthesis options */
	public static val SynthesisOption SHOW_ALL_REACTORS = SynthesisOption.createCheckOption("All Reactors", false)
	public static val SynthesisOption SHOW_REACTION_CODE = SynthesisOption.createCheckOption("Reaction Code", false)
	
	public static val SynthesisOption PAPER_MODE = SynthesisOption.createCheckOption("Paper Mode", false).setCategory(APPEARANCE)
	public static val SynthesisOption REACTIONS_USE_HYPEREDGES = SynthesisOption.createCheckOption("Bundled Dependencies", false).setCategory(APPEARANCE)
	public static val SynthesisOption USE_ALTERNATIVE_DASH_PATTERN = SynthesisOption.createCheckOption("Alternative Dependency Line Style", false).setCategory(APPEARANCE)
	public static val SynthesisOption SHOW_INSTANCE_NAMES = SynthesisOption.createCheckOption("Reactor Instance Names", false).setCategory(APPEARANCE)
	public static val SynthesisOption REACTOR_PARAMETER_MODE = SynthesisOption.createChoiceOption("Reactor Parameters", ReactorParameterDisplayModes.values, ReactorParameterDisplayModes.NONE).setCategory(APPEARANCE)
	public static val SynthesisOption REACTOR_PARAMETER_TABLE_COLS = SynthesisOption.createRangeOption("Reactor Parameter Table Columns", 1, 10, 1).setCategory(APPEARANCE)
	
	public static val SynthesisOption SHOW_REACTION_ORDER_EDGES = SynthesisOption.createCheckOption("Reaction Order Edges", false).setCategory(EXPERIMENTAL)
	public static val SynthesisOption CYCLE_DETECTION = SynthesisOption.createCheckOption("Dependency cycle detection", false).setCategory(EXPERIMENTAL)
	
    /** Synthesis actions */
    public static val DisplayedActionData COLLAPSE_ALL = DisplayedActionData.create(CollapseAllReactorsAction.ID, "Hide all Details")
    public static val DisplayedActionData EXPAND_ALL = DisplayedActionData.create(ExpandAllReactorsAction.ID, "Show all Details")
	
	override getDisplayedSynthesisOptions() {
		return #[
			SHOW_ALL_REACTORS,
			SHOW_REACTION_CODE,
			MEMORIZE_EXPANSION_STATES,
			PAPER_MODE,
			REACTIONS_USE_HYPEREDGES,
			USE_ALTERNATIVE_DASH_PATTERN,
			SHOW_INSTANCE_NAMES,
			REACTOR_PARAMETER_MODE,
			REACTOR_PARAMETER_TABLE_COLS,
			CYCLE_DETECTION,
			SHOW_REACTION_ORDER_EDGES
		]
	}
	
    override getDisplayedActions() {
        return #[COLLAPSE_ALL, EXPAND_ALL]
    }
	
	// -------------------------------------------------------------------------
	
	override KNode transform(Model model) {
		val rootNode = createNode()

		try {
			// Find main
			val main = model.reactors.findFirst[main]
			if (main !== null) {
				rootNode.children += main.createReactorNode(true, true, null, null, null, newHashSet)
			} else {
				val messageNode = createNode()
				messageNode.addErrorMessage(TEXT_NO_MAIN_REACTOR, null)
				rootNode.children += messageNode
			}
			
			// Show all reactors
			if (main === null || SHOW_ALL_REACTORS.booleanValue) {
				val reactorNodes = newArrayList()
				for (reactor : model.reactors.filter[it !== main]) {
					reactorNodes += reactor.createReactorNode(false, main === null, null, HashBasedTable.<Instantiation, Input, KPort>create, HashBasedTable.<Instantiation, Output, KPort>create, newHashSet)
				}
				if (!reactorNodes.empty) {
					// To allow ordering, we need box layout but we also need layered layout for ports thus wrap all node
					// TODO use rect packing in the future
					reactorNodes.add(0, rootNode.children.head)
					for (entry : reactorNodes.filter[!getProperty(CoreOptions.COMMENT_BOX)].indexed) {
						rootNode.children += createNode() => [
							val node = entry.value
							children += node
							// Add comment nodes
							children += node.incomingEdges.filter[source.getProperty(CoreOptions.COMMENT_BOX)].map[source]
							
							addInvisibleContainerRendering
							setLayoutOption(CoreOptions.ALGORITHM, LayeredOptions.ALGORITHM_ID)
					        setLayoutOption(CoreOptions.PADDING, new ElkPadding(0))
					        setLayoutOption(CoreOptions.PRIORITY, reactorNodes.size - entry.key) // Order!
						]
					}
					
					rootNode.setLayoutOption(CoreOptions.ALGORITHM, BoxLayouterOptions.ALGORITHM_ID)
			        rootNode.setLayoutOption(CoreOptions.SPACING_NODE_NODE, 25.0)
				}
			}
		} catch (Exception e) {
			e.printStackTrace
			
			val messageNode = createNode()
			messageNode.addErrorMessage("Error in Diagram Synthesis", e.class.simpleName + " occurred. Could not create diagram.")
			rootNode.children += messageNode
		}

		return rootNode
	}
	
	private def Collection<KNode> createReactorNode(Reactor reactor, boolean main, boolean expandDefault, Instantiation instance, Table<Instantiation, Input, KPort> inputPortsReg, Table<Instantiation, Output, KPort> outputPortsReg, Collection<Reactor>parentReactors) {
		val node = createNode()
		val nodes = newArrayList(node)
		node.associateWith(reactor)
		node.ID = main ? "main" : reactor?.name

		val label = reactor.createReactorLabel(instance)
		val recursive = parentReactors.contains(reactor)
		if (recursive) {
			node.setProperty(RECURSIVE_INSTANTIATION, true)
		}
		parentReactors += reactor
		
		if (reactor === null) {
			node.addErrorMessage(TEXT_REACTOR_NULL, null)
		} else if (main) {
			val figure = node.addMainReactorFigure(label)
			
			if (REACTOR_PARAMETER_MODE.objectValue === ReactorParameterDisplayModes.TABLE && !reactor.parameters.empty) {
				figure.addRectangle() => [
					invisible = true
					setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 4, 0)
					horizontalAlignment = HorizontalAlignment.LEFT
					
					addParameterList(reactor.parameters)
				]
			}
		
			if (recursive) {
				nodes += node.addErrorComment(TEXT_ERROR_RECURSIVE)
				figure.errorStyle()
			} else {
				figure.addChildArea()
				node.children += reactor.transformReactorNetwork(emptyMap, emptyMap, parentReactors)
			}
			
			node.configureReactorNodeLayout()
			
			// Additional layout adjustment for main node
			node.setLayoutOption(CoreOptions.ALGORITHM, LayeredOptions.ALGORITHM_ID)
			node.setLayoutOption(CoreOptions.DIRECTION, Direction.RIGHT)
			node.setLayoutOption(CoreOptions.NODE_SIZE_CONSTRAINTS, EnumSet.of(SizeConstraint.MINIMUM_SIZE))
			node.setLayoutOption(LayeredOptions.NODE_PLACEMENT_BK_FIXED_ALIGNMENT, FixedAlignment.BALANCED)
			node.setLayoutOption(LayeredOptions.NODE_PLACEMENT_BK_EDGE_STRAIGHTENING, EdgeStraighteningStrategy.IMPROVE_STRAIGHTNESS)
			node.setLayoutOption(LayeredOptions.SPACING_EDGE_NODE, LayeredOptions.SPACING_EDGE_NODE.^default * 1.1f)
			node.setLayoutOption(LayeredOptions.SPACING_EDGE_NODE_BETWEEN_LAYERS, LayeredOptions.SPACING_EDGE_NODE_BETWEEN_LAYERS.^default * 1.1f)
			if (PAPER_MODE.booleanValue) {
				node.setLayoutOption(CoreOptions.PADDING, new ElkPadding(-1, 6, 6, 6))
				node.setLayoutOption(LayeredOptions.SPACING_COMPONENT_COMPONENT, LayeredOptions.SPACING_COMPONENT_COMPONENT.^default * 0.5f)
			}
		} else {
			// Expanded Rectangle
			node.addReactorFigure(reactor, label) => [
				associateWith(reactor)
				setProperty(KlighdProperties.EXPANDED_RENDERING, true)
				addDoubleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)

				if (!PAPER_MODE.booleanValue) {
					// Collapse button
					addTextButton(TEXT_HIDE_ACTION) => [
						setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 0, 0)
						addSingleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)
						addDoubleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)
					]
				}
				
				if (REACTOR_PARAMETER_MODE.objectValue === ReactorParameterDisplayModes.TABLE && !reactor.parameters.empty) {
					addRectangle() => [
						invisible = true
						if (PAPER_MODE.booleanValue) {
							setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 4, 0)
						} else {
							setGridPlacementData().from(LEFT, 8, 0, TOP, 4, 0).to(RIGHT, 8, 0, BOTTOM, 0, 0)
						}
						horizontalAlignment = HorizontalAlignment.LEFT
						
						addParameterList(reactor.parameters)
					]
				}
				
				if (recursive) {
					errorStyle()
				} else {
					addChildArea()
				}
			]

			// Collapse Rectangle
			node.addReactorFigure(reactor, label) => [
				associateWith(reactor)
				setProperty(KlighdProperties.COLLAPSED_RENDERING, true)
				if (reactor.hasContent && !recursive) {
					addDoubleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)
				}

				if (!PAPER_MODE.booleanValue) {
					// Expand button
					if (reactor.hasContent && !recursive) {
						addTextButton(TEXT_SHOW_ACTION) => [
							setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 8, 0)
							addSingleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)
							addDoubleClickAction(MEM_EXPAND_COLLAPSE_ACTION_ID)
						]
					}
				}
				
				if (recursive) {
					errorStyle()
				}
			]
			
			// Create ports
			val inputPorts = <Input, KPort>newHashMap
			val outputPorts = <Output, KPort>newHashMap
			for (input : reactor.inputs.reverseView) {
				inputPorts.put(input, node.addIOPort(input, true))
			}
			for (output : reactor.outputs) {
				outputPorts.put(output, node.addIOPort(output, false))
			}

			// Add content
			if (reactor.hasContent && !recursive) {
				node.children += reactor.transformReactorNetwork(inputPorts, outputPorts, parentReactors)
			}
			
			// Pass port to given tables
			if (instance !== null) {
				if (inputPortsReg !== null) {
					for (entry : inputPorts.entrySet) {
						inputPortsReg.put(instance, entry.key, entry.value)
					}
				}
				if (outputPortsReg !== null) {
					for (entry : outputPorts.entrySet) {
						outputPortsReg.put(instance, entry.key, entry.value)
					}
				}
			}
			
			node.setProperty(REACTOR_INSTANCE, instance) // save to distinguish nodes associated with the same reactor
			
			if (recursive) {
				node.setLayoutOption(KlighdProperties.EXPAND, false)
				nodes += node.addErrorComment(TEXT_ERROR_RECURSIVE)
			} else {
				node.setLayoutOption(KlighdProperties.EXPAND, expandDefault)
			}
			
			node.configureReactorNodeLayout()
		}
		
		// Find and annotate cycles
		if (reactor !== null && instance === null && CYCLE_DETECTION.booleanValue) {
			val errNode = node.detectAndAnnotateCycles()
			if (errNode !== null) {
				nodes += errNode
			}
		}
		
		parentReactors -= reactor
		return nodes
	}
	
	private def configureReactorNodeLayout(KNode node) {
		node.setLayoutOption(CoreOptions.NODE_SIZE_CONSTRAINTS, SizeConstraint.minimumSizeWithPorts)
		node.setLayoutOption(CoreOptions.PORT_CONSTRAINTS, PortConstraints.FIXED_ORDER)
		node.setLayoutOption(LayeredOptions.CROSSING_MINIMIZATION_SEMI_INTERACTIVE, true)
		if (PAPER_MODE.booleanValue) {
			node.setLayoutOption(CoreOptions.PADDING, new ElkPadding(2, 6, 6, 6))
			node.setLayoutOption(LayeredOptions.SPACING_NODE_NODE, LayeredOptions.SPACING_NODE_NODE.^default * 0.75f)
			node.setLayoutOption(LayeredOptions.SPACING_NODE_NODE_BETWEEN_LAYERS, LayeredOptions.SPACING_NODE_NODE_BETWEEN_LAYERS.^default * 0.75f)
			node.setLayoutOption(LayeredOptions.SPACING_EDGE_NODE, LayeredOptions.SPACING_EDGE_NODE.^default * 0.75f)
			node.setLayoutOption(LayeredOptions.SPACING_EDGE_NODE_BETWEEN_LAYERS, LayeredOptions.SPACING_EDGE_NODE_BETWEEN_LAYERS.^default * 0.75f)
		}
	}
	
	private def KNode detectAndAnnotateCycles(KNode node) {
		try {
			var containsRecursion = false
			var depGraph = new AnnotatedDependencyGraph()
			
        	// Some edges do not specify ports hence we introduce virtual ports to correctly handle dependency separation
        	val virtualPorts = HashBiMap.<KNode,KPort>create
			
			// Build graph and find cycles
            for (childNode : node.getKNodeIterator(false).toIterable) {
            	if (childNode.getProperty(RECURSIVE_INSTANTIATION)) {
            		containsRecursion = true
            	}
            	
            	if (!childNode.outgoingEdges.empty && !childNode.incomingEdges.empty) { // simplify dependency graph
            		val vPort = virtualPorts.getOrInit(childNode)[createPort()]
            		val breaksCylce = switch(childNode.getProperty(KlighdInternalProperties.MODEL_ELEMEMT)) {
            			Action: true
            			default: false
            		}
	            	if (childNode.ports.empty) {
	            		// If node has no ports ignore outgoing if it breaks cycles
		            	if (!breaksCylce) {
			                depGraph.addEdges(new AnnotatedNode(vPort), childNode.outgoingEdges.map[
			                	new AnnotatedNode(targetPort?:virtualPorts.getOrInit(target)[createPort()])
			                ].toSet)
		            	}
	            	} else {
	            		// If node has port prefer dependencies on ports
	            		for (edge : childNode.outgoingEdges.filter[!target.outgoingEdges.empty]) { // also simplify dependency graph
	            			depGraph.addEdge(
		                		new AnnotatedNode(edge.sourcePort?:vPort),
		                		new AnnotatedNode(edge.targetPort?:virtualPorts.getOrInit(edge.target)[createPort()])
	            			)
	            		}
	            		// If node breaks cycles or has children -> do not introduce dependencies from input to output
		            	if (!breaksCylce && childNode.children.empty) {
		            		val vPortNode = new AnnotatedNode(vPort)
		            		val outputs = childNode.outgoingEdges.map[sourcePort].filterNull.toSet.map[new AnnotatedNode(it)]
		            		val inputs = childNode.incomingEdges.map[targetPort].filterNull.toSet.map[new AnnotatedNode(it)]
		            		
		            		for (input : inputs) {
	            				depGraph.addEdge(input, vPortNode)
		            		}
	            			depGraph.addEdges(vPortNode, outputs.toSet)
		            	}
	            	}
            	}
            }
            depGraph.detectCycles()
            
            if (DEBUG_CYCLE_DETECTION) {
	            println("-- DEBUG CYCLE DETECTION --")
	            for (n : depGraph.nodes) {
	            	for(d : depGraph.getDependencies(n)) {
	            		val sN = (n.contents.eContainer?:virtualPorts.inverse.get(n.contents)) as KNode
	            		val tN = (d.contents.eContainer?:virtualPorts.inverse.get(d.contents)) as KNode
	            		val sO = sN?.getProperty(KlighdInternalProperties.MODEL_ELEMEMT).toString
	            		val tO = tN?.getProperty(KlighdInternalProperties.MODEL_ELEMEMT).toString
	            		val sP = n.contents.eContainer === null ? "virtual" : n.contents.getProperty(CoreOptions.PORT_SIDE).toString
	            		val tP = d.contents.eContainer === null ? "virtual" : d.contents.getProperty(CoreOptions.PORT_SIDE).toString
	            		println("%s[%s] -> %s[%s]".format(sO, sP, tO, tP))
	            	}
	            }
			}
            
            if (!depGraph.cycles.empty) {
    			// Highlight cycles
                for (cycle : depGraph.cycles) {
                	val cyclePorts = cycle.map[contents].toSet
                	val cycleNodes = cycle.map[contents].map[
                		if (it.node !== null) {
                			it.node
                		} else {
                			virtualPorts.inverse.get(it)
                		}
                	].filterNull.toSet
                	for (cycleNode : cycleNodes) {
                		cycleNode.setProperty(DEPENDENCY_CYCLE, true)
                		cycleNode.data.filter(KRendering).forEach[errorStyle()]
                		for (cycleEgde : cycleNode.outgoingEdges.filter[
                			cycleNodes.contains(target) &&
                			sourcePort === null ? true : cyclePorts.contains(sourcePort)
                		]) {
                			cycleEgde.setProperty(DEPENDENCY_CYCLE, true)
                			cycleEgde.data.filter(KRendering).forEach[errorStyle()]
                			// TODO initiallyHie does not work with incremental
                			// cycleEgde.initiallyShow() // Show hidden order dependencies
							cycleEgde.KRendering.invisible = false
                		}
                	}
                }
                
                val err = node.addErrorComment(TEXT_ERROR_CONTAINS_CYCLE)
                err.KContainerRendering.addRectangle() => [ // Add to existing figure
                	setGridPlacementData().from(LEFT, 3, 0, TOP, -1, 0).to(RIGHT, 3, 0, BOTTOM, 3, 0)
            		noSelectionStyle()
            		 //addSingleClickAction(ShowCycleAction.ID) FIXME: causes build error
            		
            		addText(TEXT_ERROR_CYCLE_BTN_SHOW) => [
            			styles += err.KContainerRendering.children.head.styles.map[copy] // Copy text style
		            	fontSize = 5
            			setSurroundingSpace(1, 0)
            			noSelectionStyle()
            			// addSingleClickAction(ShowCycleAction.ID) // FIXME: causes build error
            		]
                ]
                if (containsRecursion) {
                	err.KContainerRendering.addText(TEXT_ERROR_CONTAINS_RECURSION) => [ // Add to existing figure
                		styles += err.KContainerRendering.children.head.styles.map[copy] // Copy text style
                		setGridPlacementData().from(LEFT, 3, 0, TOP, 0, 0).to(RIGHT, 3, 0, BOTTOM, 3, 0)
                	]
                }
                
                return err
            } else if (containsRecursion) {
            	return node.addErrorComment(TEXT_ERROR_CONTAINS_RECURSION)
            }
        } catch(Exception e) {
        	e.printStackTrace()
        	return node.addErrorComment(TEXT_ERROR_CYCLE_DETECTION)
        }
	}

	private def Collection<KNode> transformReactorNetwork(Reactor reactor, Map<Input, KPort> parentInputPorts, Map<Output, KPort> parentOutputPorts, Collection<Reactor>parentReactors) {
		val nodes = <KNode>newArrayList
		val inputPorts = HashBasedTable.<Instantiation, Input, KPort>create
		val outputPorts = HashBasedTable.<Instantiation, Output, KPort>create
		val reactionNodes = <Reaction, KNode>newHashMap
		val actionDestinations = HashMultimap.<Action, KPort>create
		val actionSources = HashMultimap.<Action, KPort>create
		val timerNodes = <Timer, KNode>newHashMap
		val startupNode = createNode
		var startupUsed = false
		val shutdownNode = createNode
		var shutdownUsed = false

		// Transform instances
		for (entry : reactor.instantiations.indexed) {
			val instance = entry.value
			val node = instance.reactorClass.createReactorNode(false, instance.getExpansionState?:false, instance, inputPorts, outputPorts, parentReactors)
			node.head.setLayoutOption(CoreOptions.PRIORITY, reactor.instantiations.size - entry.key)
			nodes += node
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
			val idx = reactor.reactions.indexOf(reaction)
			val node = createNode().associateWith(reaction)
			nodes += node
			reactionNodes.put(reaction, node)
			
			node.setLayoutOption(CoreOptions.PORT_CONSTRAINTS, PortConstraints.FIXED_SIDE)
			node.setLayoutOption(CoreOptions.PRIORITY, (reactor.reactions.size - idx) * 10 ) // always place with higher priority than reactor nodes
			node.setLayoutOption(LayeredOptions.POSITION, new KVector(0, idx)) // try order reactions vertically if in one layer
			
			node.addReactionFigure(reaction)
		
			// connect input
			var KPort port
			for (TriggerRef trigger : reaction.triggers?:emptyList) {
				port = if (REACTIONS_USE_HYPEREDGES.booleanValue && port !== null) {
					port
				} else {
					node.addInvisiblePort() => [
						setLayoutOption(CoreOptions.PORT_SIDE, PortSide.WEST)
						if (REACTIONS_USE_HYPEREDGES.booleanValue || ((reaction.triggers?:emptyList).size + (reaction.sources?:emptyList).size) == 1) {
							setLayoutOption(CoreOptions.PORT_BORDER_OFFSET, -LinguaFrancaShapeExtensions.REACTION_POINTINESS as double) // manual adjustment disabling automatic one
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
			//port = null // create new ports
			for (VarRef dep : reaction.sources?:emptyList) {
				port = if (REACTIONS_USE_HYPEREDGES.booleanValue && port !== null) {
					port
				} else {
					node.addInvisiblePort() => [
						//setLayoutOption(CoreOptions.PORT_SIDE, PortSide.NORTH)
						setLayoutOption(CoreOptions.PORT_SIDE, PortSide.WEST)
						if (REACTIONS_USE_HYPEREDGES.booleanValue || ((reaction.triggers?:emptyList).size + (reaction.sources?:emptyList).size) == 1) {
							setLayoutOption(CoreOptions.PORT_BORDER_OFFSET, -LinguaFrancaShapeExtensions.REACTION_POINTINESS as double)  // manual adjustment disabling automatic one
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
			port = null // create new ports
			for (VarRef effect : reaction.effects?:emptyList) {
				port = if (REACTIONS_USE_HYPEREDGES.booleanValue && port !== null) {
					port
				} else {
					node.addInvisiblePort() => [
						setLayoutOption(CoreOptions.PORT_SIDE, PortSide.EAST)
					]
				}
				if (effect.variable instanceof Action) {
					actionSources.put(effect.variable as Action, port)
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
		val actions = newHashSet
		actions += actionSources.keySet
		actions += actionDestinations.keySet
		for (Action action : actions) {
			val node = createNode().associateWith(action)
			nodes += node
			node.setLayoutOption(CoreOptions.PORT_CONSTRAINTS, PortConstraints.FIXED_SIDE)
			
			val ports = node.addActionFigureAndPorts(action.origin === ActionOrigin.PHYSICAL ? "P" : "L")
			if (action.minDelay !== null) {
				node.addOutsideBottomCenteredNodeLabel(action.minDelay.toText, 7)
			}
			
			// connect source
			for (source : actionSources.get(action)) {
				createDelayEdge(action).connect(source, ports.key)
			}
			
			// connect targets
			for (target : actionDestinations.get(action)) {
				createDelayEdge(action).connect(ports.value, target)
			}
		}

		// Transform connections
		for (Connection connection : reactor.connections?:emptyList) {
			val source = if (connection.leftPort?.container !== null) {
				outputPorts.get(connection.leftPort.container, connection.leftPort.variable)
			} else if (parentInputPorts.containsKey(connection.leftPort?.variable)) {
				parentInputPorts.get(connection.leftPort.variable)
			}
			val target = if (connection.rightPort?.container !== null) {
				inputPorts.get(connection.rightPort.container, connection.rightPort.variable)
			} else if (parentOutputPorts.containsKey(connection.rightPort?.variable)) {
				parentOutputPorts.get(connection.rightPort.variable)
			}
			val edge = createIODependencyEdge(connection)
			if (connection.delay !== null) {
				edge.addCenterEdgeLabel(connection.delay.toText) => [
					associateWith(connection.delay)
					applyOnEdgeDelayStyle()
				]
			}
			if (source !== null && target !== null) {
				edge.connect(source, target)
			}
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
		
		// Add reaction order edges (add last to have them on top of other edges)
		if (reactor.reactions.size > 1) {
			var prevNode = reactionNodes.get(reactor.reactions.head)
			for (node : reactor.reactions.drop(1).map[reactionNodes.get(it)]) {
				val edge = createOrderEdge()
				edge.source = prevNode
				edge.target = node
				edge.setProperty(CoreOptions.NO_LAYOUT, true)
				
				// Do not remove them, as they are needed for cycle detection
				edge.KRendering.invisible = !SHOW_REACTION_ORDER_EDGES.booleanValue
				edge.KRendering.invisible.propagateToChildren = true
				// TODO this does not work work with incremental update -> check for bug
				// if (!SHOW_REACTION_ORDER_EDGES.booleanValue) edge.initiallyHide()
				
				prevNode = node
			}
		}
		
		return nodes
	}
	
	private def String createReactorLabel(Reactor reactor, Instantiation instance) {
		val b = new StringBuilder
		if (SHOW_INSTANCE_NAMES.booleanValue && instance !== null) {
			b.append(instance.name).append(" : ")
		}
		b.append(reactor === null ? "<NULL>" : reactor.name?:"<Unresolved Reactor>")
		if (REACTOR_PARAMETER_MODE.objectValue === ReactorParameterDisplayModes.TITLE && reactor !== null) {
			if (reactor.parameters.empty) {
				b.append("()")
			} else {
				b.append(reactor.parameters.join("(", ", ", ")")[createParameterLabel(false)])
			}
		}
		return b.toString()
	}
	
	private def addParameterList(KContainerRendering container, List<Parameter> parameters) {
		var cols = 1
		try {
			cols = REACTOR_PARAMETER_TABLE_COLS.intValue
		} catch (Exception e) {} // ignore
		if (cols > parameters.size) {
			cols = parameters.size
		}
		container.gridPlacement = cols
		for (param : parameters) {
			container.addText(param.createParameterLabel(true)) => [
				fontSize = 8
				horizontalAlignment = HorizontalAlignment.LEFT
			]
		}
	}
	
	private def String createParameterLabel(Parameter param, boolean bullet) {
		val b = new StringBuilder
		if (bullet) {
			b.append("\u2022 ")
		}
		b.append(param.name)
		if (!ASTUtils.toText(param.type, null).nullOrEmpty) {
			b.append(":").append(param.type)
		} else if (ASTUtils.isOfTimeType(param)) {
			b.append(":time")
		}
		return b.toString()
	}
	
	private def createDelayEdge(Object associate) {
		return createEdge => [
			associateWith(associate)
			addPolyline() => [
				if (USE_ALTERNATIVE_DASH_PATTERN.booleanValue) {
					lineStyle = LineStyle.CUSTOM
					lineStyle.dashPattern += ALTERNATIVE_DASH_PATTERN
				} else {
					lineStyle = LineStyle.DASH
				}
				boldLineSelectionStyle()
			]
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
				if (USE_ALTERNATIVE_DASH_PATTERN.booleanValue) {
					lineStyle = LineStyle.CUSTOM
					lineStyle.dashPattern += ALTERNATIVE_DASH_PATTERN
				} else {
					lineStyle = LineStyle.DASH
				}
				boldLineSelectionStyle()
			]
		]
	}
	
	private def createOrderEdge() {
		return createEdge => [
			addPolyline() => [
				lineWidth = 1.5f
				lineStyle = LineStyle.DOT
				foreground = Colors.CHOCOLATE_1
				boldLineSelectionStyle()
				addHeadArrowDecorator()
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
			port.setLayoutOption(CoreOptions.PORT_SIDE, PortSide.WEST)
			port.setLayoutOption(CoreOptions.PORT_BORDER_OFFSET, -3.0)
		} else {
			port.setLayoutOption(CoreOptions.PORT_SIDE, PortSide.EAST)
			port.setLayoutOption(CoreOptions.PORT_BORDER_OFFSET, -3.0)
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
	
	private def KNode addErrorComment(KNode node, String message) {
		val comment = createNode()
        comment.setLayoutOption(CoreOptions.COMMENT_BOX, true)
        comment.addCommentFigure(message) => [
        	errorStyle()
        	background = Colors.PEACH_PUFF_2
        ]
        
        // connect
        createEdge() => [
        	source = comment
        	target = node
        	addCommentPolyline().errorStyle()
        ]  
        
        return comment
	}

}
