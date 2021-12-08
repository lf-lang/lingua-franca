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
package org.lflang.diagram.synthesis

import com.google.common.collect.HashBasedTable
import com.google.common.collect.HashMultimap
import com.google.common.collect.Table
import de.cau.cs.kieler.klighd.DisplayedActionData
import de.cau.cs.kieler.klighd.SynthesisOption
import de.cau.cs.kieler.klighd.kgraph.KEdge
import de.cau.cs.kieler.klighd.kgraph.KNode
import de.cau.cs.kieler.klighd.kgraph.KPort
import de.cau.cs.kieler.klighd.krendering.Colors
import de.cau.cs.kieler.klighd.krendering.HorizontalAlignment
import de.cau.cs.kieler.klighd.krendering.KContainerRendering
import de.cau.cs.kieler.klighd.krendering.KRendering
import de.cau.cs.kieler.klighd.krendering.LineCap
import de.cau.cs.kieler.klighd.krendering.LineStyle
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared
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
import org.eclipse.elk.alg.layered.options.EdgeStraighteningStrategy
import org.eclipse.elk.alg.layered.options.FixedAlignment
import org.eclipse.elk.alg.layered.options.LayerConstraint
import org.eclipse.elk.alg.layered.options.LayeredOptions
import org.eclipse.elk.core.math.ElkMargin
import org.eclipse.elk.core.math.ElkPadding
import org.eclipse.elk.core.math.KVector
import org.eclipse.elk.core.options.BoxLayouterOptions
import org.eclipse.elk.core.options.CoreOptions
import org.eclipse.elk.core.options.Direction
import org.eclipse.elk.core.options.PortConstraints
import org.eclipse.elk.core.options.PortSide
import org.eclipse.elk.core.options.SizeConstraint
import org.eclipse.elk.graph.properties.Property
import org.eclipse.emf.ecore.EObject
import org.lflang.ASTUtils
import org.lflang.FileConfig
import org.lflang.diagram.synthesis.action.CollapseAllReactorsAction
import org.lflang.diagram.synthesis.action.ExpandAllReactorsAction
import org.lflang.diagram.synthesis.action.FilterCycleAction
import org.lflang.diagram.synthesis.action.MemorizingExpandCollapseAction
import org.lflang.diagram.synthesis.action.ShowCycleAction
import org.lflang.diagram.synthesis.styles.LinguaFrancaShapeExtensions
import org.lflang.diagram.synthesis.styles.LinguaFrancaStyleExtensions
import org.lflang.diagram.synthesis.styles.ReactorFigureComponents
import org.lflang.diagram.synthesis.util.CycleVisualization
import org.lflang.diagram.synthesis.util.InterfaceDependenciesVisualization
import org.lflang.diagram.synthesis.util.ReactorIcons
import org.lflang.diagram.synthesis.util.SynthesisErrorReporter
import org.lflang.diagram.synthesis.util.UtilityExtensions
import org.lflang.generator.ActionInstance
import org.lflang.generator.ParameterInstance
import org.lflang.generator.PortInstance
import org.lflang.generator.ReactionInstance
import org.lflang.generator.ReactorInstance
import org.lflang.generator.TimerInstance
import org.lflang.generator.TriggerInstance
import org.lflang.generator.TriggerInstance.BuiltinTriggerVariable
import org.lflang.lf.Connection
import org.lflang.lf.Model

import static extension org.eclipse.emf.ecore.util.EcoreUtil.*
import static extension org.lflang.ASTUtils.*
import static extension org.lflang.diagram.synthesis.action.MemorizingExpandCollapseAction.*
import static extension org.lflang.diagram.synthesis.util.NamedInstanceUtil.*

/**
 * Diagram synthesis for Lingua Franca programs.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
@ViewSynthesisShared
class LinguaFrancaSynthesis extends AbstractDiagramSynthesis<Model> {

	@Inject extension KNodeExtensions
	@Inject extension KEdgeExtensions
	@Inject extension KPortExtensions
	@Inject extension KLabelExtensions
	@Inject extension KRenderingExtensions
	@Inject extension KContainerRenderingExtensions
	@Inject extension KPolylineExtensions
	@Inject extension LinguaFrancaStyleExtensions
	@Inject extension LinguaFrancaShapeExtensions
	@Inject extension UtilityExtensions
	@Inject extension CycleVisualization
    @Inject extension InterfaceDependenciesVisualization
	@Inject extension FilterCycleAction
	@Inject extension ReactorIcons
	
	// -------------------------------------------------------------------------
	
	public static val ID = "org.lflang.diagram.synthesis.LinguaFrancaSynthesis"

	// -- INTERNAL --
	public static val REACTOR_RECURSIVE_INSTANTIATION = new Property<Boolean>("org.lflang.linguafranca.diagram.synthesis.reactor.recursive.instantiation", false)
    public static val REACTOR_HAS_BANK_PORT_OFFSET = new Property<Boolean>("org.lflang.linguafranca.diagram.synthesis.reactor.bank.offset", false)
    public static val REACTOR_INPUT = new Property<Boolean>("org.lflang.linguafranca.diagram.synthesis.reactor.input", false)
    public static val REACTOR_OUTPUT = new Property<Boolean>("org.lflang.linguafranca.diagram.synthesis.reactor.output", false)
	
	// -- STYLE --	
	public static val ALTERNATIVE_DASH_PATTERN = #[3.0f]
	
	// -- TEXT --
	public static val TEXT_ERROR_RECURSIVE = "Recursive reactor instantiation!"
	public static val TEXT_ERROR_CONTAINS_RECURSION = "Reactor contains recursive instantiation!"
	public static val TEXT_ERROR_CONTAINS_CYCLE = "Reactor contains cyclic dependencies!"
	public static val TEXT_ERROR_CYCLE_DETECTION = "Dependency cycle detection failed.\nCould not detect dependency cycles due to unexpected graph structure."
	public static val TEXT_ERROR_CYCLE_BTN_SHOW = "Show Cycle"
	public static val TEXT_ERROR_CYCLE_BTN_FILTER = "Filter Cycle"
	public static val TEXT_ERROR_CYCLE_BTN_UNFILTER = "Remove Cycle Filter"
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
	public static val SynthesisOption CYCLE_DETECTION = SynthesisOption.createCheckOption("Dependency Cycle Detection", true)
	
	public static val SynthesisOption SHOW_USER_LABELS = SynthesisOption.createCheckOption("User Labels (@label in JavaDoc)", true).setCategory(APPEARANCE)
	public static val SynthesisOption SHOW_HYPERLINKS = SynthesisOption.createCheckOption("Expand/Collapse Hyperlinks", false).setCategory(APPEARANCE)
	public static val SynthesisOption REACTIONS_USE_HYPEREDGES = SynthesisOption.createCheckOption("Bundled Dependencies", false).setCategory(APPEARANCE)
	public static val SynthesisOption USE_ALTERNATIVE_DASH_PATTERN = SynthesisOption.createCheckOption("Alternative Dependency Line Style", false).setCategory(APPEARANCE)
    public static val SynthesisOption SHOW_PORT_NAMES = SynthesisOption.createCheckOption("Port names", true).setCategory(APPEARANCE)
    public static val SynthesisOption SHOW_MULTIPORT_WIDTH = SynthesisOption.createCheckOption("Multiport Widths", false).setCategory(APPEARANCE)
	public static val SynthesisOption SHOW_REACTION_CODE = SynthesisOption.createCheckOption("Reaction Code", false).setCategory(APPEARANCE)
    public static val SynthesisOption SHOW_REACTION_LEVEL = SynthesisOption.createCheckOption("Reaction Level", false).setCategory(APPEARANCE)
	public static val SynthesisOption SHOW_REACTION_ORDER_EDGES = SynthesisOption.createCheckOption("Reaction Order Edges", false).setCategory(APPEARANCE)
	public static val SynthesisOption SHOW_REACTOR_HOST = SynthesisOption.createCheckOption("Reactor Host Addresses", true).setCategory(APPEARANCE)
	public static val SynthesisOption SHOW_INSTANCE_NAMES = SynthesisOption.createCheckOption("Reactor Instance Names", false).setCategory(APPEARANCE)
	public static val SynthesisOption REACTOR_PARAMETER_MODE = SynthesisOption.createChoiceOption("Reactor Parameters", ReactorParameterDisplayModes.values, ReactorParameterDisplayModes.NONE).setCategory(APPEARANCE)
	public static val SynthesisOption REACTOR_PARAMETER_TABLE_COLS = SynthesisOption.createRangeOption("Reactor Parameter Table Columns", 1, 10, 1).setCategory(APPEARANCE)
	
    /** Synthesis actions */
    public static val DisplayedActionData COLLAPSE_ALL = DisplayedActionData.create(CollapseAllReactorsAction.ID, "Hide all Details")
    public static val DisplayedActionData EXPAND_ALL = DisplayedActionData.create(ExpandAllReactorsAction.ID, "Show all Details")
	
	override getDisplayedSynthesisOptions() {
		return #[
			SHOW_ALL_REACTORS,
			MEMORIZE_EXPANSION_STATES,
			CYCLE_DETECTION,
			SHOW_USER_LABELS,
			SHOW_HYPERLINKS,
			//LinguaFrancaSynthesisInterfaceDependencies.SHOW_INTERFACE_DEPENDENCIES,
			REACTIONS_USE_HYPEREDGES,
			USE_ALTERNATIVE_DASH_PATTERN,
			SHOW_PORT_NAMES,
			SHOW_MULTIPORT_WIDTH,
			SHOW_REACTION_CODE,
            SHOW_REACTION_LEVEL,
			SHOW_REACTION_ORDER_EDGES,
			SHOW_REACTOR_HOST,
			SHOW_INSTANCE_NAMES,
			REACTOR_PARAMETER_MODE,
			REACTOR_PARAMETER_TABLE_COLS
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
			val main = model.reactors.findFirst[isMainOrFederated]
			if (main !== null) {
			    val reactorInstance = new ReactorInstance(main, new SynthesisErrorReporter())
				rootNode.children += reactorInstance.createReactorNode(true, null, null, newHashMap)
			} else {
				val messageNode = createNode()
				messageNode.addErrorMessage(TEXT_NO_MAIN_REACTOR, null)
				rootNode.children += messageNode
			}
			
			// Show all reactors
			if (main === null || SHOW_ALL_REACTORS.booleanValue) {
				val reactorNodes = newArrayList()
				for (reactor : model.reactors.filter[it !== main]) {
				    val reactorInstance = new ReactorInstance(reactor, new SynthesisErrorReporter(), emptySet)
					reactorNodes += reactorInstance.createReactorNode(main === null, HashBasedTable.<ReactorInstance, PortInstance, KPort>create, HashBasedTable.<ReactorInstance, PortInstance, KPort>create, newHashMap)
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
	
	private def Collection<KNode> createReactorNode(
		ReactorInstance reactorInstance,
		boolean expandDefault,
		Table<ReactorInstance, PortInstance, KPort> inputPortsReg,
		Table<ReactorInstance, PortInstance, KPort> outputPortsReg,
		Map<ReactorInstance, KNode> allReactorNodes
	) {
		val reactor = reactorInstance.reactorDefinition
		
		val node = createNode()
        allReactorNodes.put(reactorInstance, node)
		node.associateWith(reactor)
		node.ID = reactorInstance.uniqueID
		node.linkInstance(reactorInstance) // save to distinguish nodes associated with the same reactor
        
		val nodes = newArrayList(node)
		val label = reactorInstance.createReactorLabel()

		if (reactorInstance.recursive) {
			// Mark this node
			node.setProperty(REACTOR_RECURSIVE_INSTANTIATION, true)
			// Mark root
			allReactorNodes.get(reactorInstance.root()).setProperty(REACTOR_RECURSIVE_INSTANTIATION, true)
		}
		
		if (reactor === null) {
			node.addErrorMessage(TEXT_REACTOR_NULL, null)
		} else if (reactorInstance.mainOrFederated) {
			val figure = node.addMainReactorFigure(reactorInstance, label)
			
			if (REACTOR_PARAMETER_MODE.objectValue === ReactorParameterDisplayModes.TABLE 
			    && !reactorInstance.parameters.empty
			) {
				figure.addRectangle() => [
					invisible = true
					setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 4, 0)
					horizontalAlignment = HorizontalAlignment.LEFT
					
					addParameterList(reactorInstance.parameters)
				]
			}

			if (reactorInstance.recursive) {
				nodes += node.addErrorComment(TEXT_ERROR_RECURSIVE)
				figure.errorStyle()
			} else {
				figure.addChildArea()
				node.children += reactorInstance.transformReactorNetwork(emptyMap, emptyMap, allReactorNodes)
			}
			
			nodes += reactor.createUserComments(node)
			
			node.configureReactorNodeLayout()
			
			// Additional layout adjustment for main node
			node.setLayoutOption(CoreOptions.ALGORITHM, LayeredOptions.ALGORITHM_ID)
			node.setLayoutOption(CoreOptions.DIRECTION, Direction.RIGHT)
			node.setLayoutOption(CoreOptions.NODE_SIZE_CONSTRAINTS, EnumSet.of(SizeConstraint.MINIMUM_SIZE))
			node.setLayoutOption(LayeredOptions.NODE_PLACEMENT_BK_FIXED_ALIGNMENT, FixedAlignment.BALANCED)
			node.setLayoutOption(LayeredOptions.NODE_PLACEMENT_BK_EDGE_STRAIGHTENING, EdgeStraighteningStrategy.IMPROVE_STRAIGHTNESS)
			node.setLayoutOption(LayeredOptions.SPACING_EDGE_NODE, LayeredOptions.SPACING_EDGE_NODE.^default * 1.1f)
			node.setLayoutOption(LayeredOptions.SPACING_EDGE_NODE_BETWEEN_LAYERS, LayeredOptions.SPACING_EDGE_NODE_BETWEEN_LAYERS.^default * 1.1f)
			if (!SHOW_HYPERLINKS.booleanValue) {
				node.setLayoutOption(CoreOptions.PADDING, new ElkPadding(-1, 6, 6, 6))
				node.setLayoutOption(LayeredOptions.SPACING_COMPONENT_COMPONENT, LayeredOptions.SPACING_COMPONENT_COMPONENT.^default * 0.5f)
			}
		} else {
            // If the reactor is a bank, then obtain the details from the first
            // element of the bank rather than the bank itself.
            val instance = if (reactorInstance.bankSize > 0) {
                reactorInstance.bankMembers.get(0)
            } else {
                reactorInstance
            }
            
			// Expanded Rectangle
			node.addReactorFigure(reactorInstance, label) => [ ReactorFigureComponents comps |
				comps.figures.forEach[associateWith(reactor)]
				comps.outer.setProperty(KlighdProperties.EXPANDED_RENDERING, true)
				comps.figures.forEach[addDoubleClickAction(MemorizingExpandCollapseAction.ID)]
				comps.reactor.handleIcon(reactor, false)

				if (SHOW_HYPERLINKS.booleanValue) {
					// Collapse button
					comps.reactor.addTextButton(TEXT_HIDE_ACTION) => [
						setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 0, 0)
						addSingleClickAction(MemorizingExpandCollapseAction.ID)
						addDoubleClickAction(MemorizingExpandCollapseAction.ID)
					]
				}
				
				if (REACTOR_PARAMETER_MODE.objectValue === ReactorParameterDisplayModes.TABLE 
				    && !instance.parameters.empty
				) {
					comps.reactor.addRectangle() => [
						invisible = true
						if (!SHOW_HYPERLINKS.booleanValue) {
							setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 4, 0)
						} else {
							setGridPlacementData().from(LEFT, 8, 0, TOP, 4, 0).to(RIGHT, 8, 0, BOTTOM, 0, 0)
						}
						horizontalAlignment = HorizontalAlignment.LEFT
						
						addParameterList(instance.parameters)
					]
				}
				
				if (instance.recursive) {
					comps.figures.forEach[errorStyle()]
				} else {
					comps.reactor.addChildArea()
				}
			]

			// Collapse Rectangle
			node.addReactorFigure(reactorInstance, label) => [ ReactorFigureComponents comps |
				comps.figures.forEach[associateWith(reactor)]
				comps.outer.setProperty(KlighdProperties.COLLAPSED_RENDERING, true)
				if (instance.hasContent && !instance.recursive) {
					comps.figures.forEach[addDoubleClickAction(MemorizingExpandCollapseAction.ID)]
				}
				comps.reactor.handleIcon(reactor, true)

				if (SHOW_HYPERLINKS.booleanValue) {
					// Expand button
					if (instance.hasContent && !instance.recursive) {
						comps.reactor.addTextButton(TEXT_SHOW_ACTION) => [
							setGridPlacementData().from(LEFT, 8, 0, TOP, 0, 0).to(RIGHT, 8, 0, BOTTOM, 8, 0)
							addSingleClickAction(MemorizingExpandCollapseAction.ID)
							addDoubleClickAction(MemorizingExpandCollapseAction.ID)
						]
					}
				}
				
				if (instance.recursive) {
					comps.figures.forEach[errorStyle()]
				}
			]
			
			// Create ports
			val inputPorts = <PortInstance, KPort>newHashMap
			val outputPorts = <PortInstance, KPort>newHashMap
			for (input : instance.inputs.reverseView) {
    		    inputPorts.put(input, node.addIOPort(input, true, input.isMultiport(), reactorInstance.isBank()))
			}
			for (output : instance.outputs) {
			    outputPorts.put(output, node.addIOPort(output, false, output.isMultiport(), reactorInstance.isBank()))
			}
			// Mark ports
			inputPorts.values.forEach[setProperty(REACTOR_INPUT, true)]
            outputPorts.values.forEach[setProperty(REACTOR_OUTPUT, true)]

			// Add content
			if (instance.hasContent && !instance.recursive) {
				node.children += instance.transformReactorNetwork(inputPorts, outputPorts, allReactorNodes)
			}
			
			// Pass port to given tables
			if (!instance.isRoot) {
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
			
			if (instance.recursive) {
				node.setLayoutOption(KlighdProperties.EXPAND, false)
				nodes += node.addErrorComment(TEXT_ERROR_RECURSIVE)
			} else {
				node.setLayoutOption(KlighdProperties.EXPAND, expandDefault)
				
				// Interface Dependencies
				node.addInterfaceDependencies(expandDefault)
			}
			
			if (!instance.isRoot) {
				// If all reactors are being shown, then only put the label on
				// the reactor definition, not on its instances. Otherwise,
				// add the annotation now.
				if (!SHOW_ALL_REACTORS.booleanValue) {
					nodes += reactor.createUserComments(node)
				}
			} else {
				nodes += reactor.createUserComments(node)
			}
			
			node.configureReactorNodeLayout()
		}

		// Find and annotate cycles
		if (CYCLE_DETECTION.booleanValue && reactorInstance.isRoot) {
			val errNode = node.detectAndAnnotateCycles(reactorInstance, allReactorNodes)
			if (errNode !== null) {
				nodes += errNode
			}
		}

		return nodes
	}
	
	private def configureReactorNodeLayout(KNode node) {
		node.setLayoutOption(CoreOptions.NODE_SIZE_CONSTRAINTS, SizeConstraint.minimumSizeWithPorts)
		node.setLayoutOption(CoreOptions.PORT_CONSTRAINTS, PortConstraints.FIXED_ORDER)
		node.setLayoutOption(LayeredOptions.CROSSING_MINIMIZATION_SEMI_INTERACTIVE, true)
		if (!SHOW_HYPERLINKS.booleanValue) {
			node.setLayoutOption(CoreOptions.PADDING, new ElkPadding(2, 6, 6, 6))
			node.setLayoutOption(LayeredOptions.SPACING_NODE_NODE, LayeredOptions.SPACING_NODE_NODE.^default * 0.75f)
			node.setLayoutOption(LayeredOptions.SPACING_NODE_NODE_BETWEEN_LAYERS, LayeredOptions.SPACING_NODE_NODE_BETWEEN_LAYERS.^default * 0.75f)
			node.setLayoutOption(LayeredOptions.SPACING_EDGE_NODE, LayeredOptions.SPACING_EDGE_NODE.^default * 0.75f)
			node.setLayoutOption(LayeredOptions.SPACING_EDGE_NODE_BETWEEN_LAYERS, LayeredOptions.SPACING_EDGE_NODE_BETWEEN_LAYERS.^default * 0.75f)
		}
	}
	
	private def KNode detectAndAnnotateCycles(KNode node, ReactorInstance reactorInstance, Map<ReactorInstance, KNode> allReactorNodes) {
		if (node.getProperty(REACTOR_RECURSIVE_INSTANTIATION)) {
			node.resetCycleFiltering()
    		return node.addErrorComment(TEXT_ERROR_CONTAINS_RECURSION)
		} else { // only detect dependency cycles if not recursive
			try {
				val hasCycle = reactorInstance.detectAndHighlightCycles(allReactorNodes, [
					if (it instanceof KNode) {
						val renderings = it.data.filter(typeof(KRendering)).toList
						if (renderings.size === 1) {
							renderings.head.errorStyle()
						} else {
							renderings.filter[getProperty(KlighdProperties.COLLAPSED_RENDERING)].forEach[errorStyle()]
						}
					} else if (it instanceof KEdge) {
						it.data.filter(typeof(KRendering)).forEach[errorStyle()]
	        			// TODO initiallyHide does not work with incremental (https://github.com/kieler/KLighD/issues/37)
	        			// cycleEgde.initiallyShow() // Show hidden order dependencies
						it.KRendering.invisible = false
					} else if (it instanceof KPort) {
						it.data.filter(typeof(KRendering)).forEach[errorStyle()]
						//it.reverseTrianglePort()
					}
				])
	            
	            if (hasCycle) {
	                val err = node.addErrorComment(TEXT_ERROR_CONTAINS_CYCLE)
	                err.KContainerRendering.addRectangle() => [ // Add to existing figure
	                	setGridPlacementData().from(LEFT, 3, 0, TOP, -1, 0).to(RIGHT, 3, 0, BOTTOM, 3, 0)
	            		noSelectionStyle()
	            		invisible = true
	            		gridPlacement = 2
	            		
	            		addRectangle() => [
		                	setGridPlacementData().from(LEFT, 0, 0, TOP, 0, 0).to(RIGHT, 2, 0, BOTTOM, 0, 0)
		            		noSelectionStyle()
		            		addSingleClickAction(ShowCycleAction.ID)
		            		addText(TEXT_ERROR_CYCLE_BTN_SHOW) => [
		            			styles += err.KContainerRendering.children.head.styles.map[copy] // Copy text style
				            	fontSize = 5
		            			setSurroundingSpace(1, 0)
		            			noSelectionStyle()
		            			addSingleClickAction(ShowCycleAction.ID)
		            		]
	            		]
	            		addRectangle() => [
		                	setGridPlacementData().from(LEFT, 0, 0, TOP, 0, 0).to(RIGHT, 0, 0, BOTTOM, 0, 0)
		            		noSelectionStyle()
		            		addSingleClickAction(FilterCycleAction.ID)
		            		addText(node.isCycleFiltered() ? TEXT_ERROR_CYCLE_BTN_UNFILTER : TEXT_ERROR_CYCLE_BTN_FILTER) => [
		            			styles += err.KContainerRendering.children.head.styles.map[copy] // Copy text style
				            	fontSize = 5
		            			setSurroundingSpace(1, 0)
		            			noSelectionStyle()
		            			addSingleClickAction(FilterCycleAction.ID)
		            			markCycleFilterText(err)
		            		]
	            		]
	                ]
	                
	                // if user interactively requested a filtered diagram keep it filtered during updates
	                if (node.isCycleFiltered()) {
	                	node.filterCycle()
	                }
	                
	                return err
	            }
			} catch(Exception e) {
	        	node.resetCycleFiltering()
	        	e.printStackTrace()
	        	return node.addErrorComment(TEXT_ERROR_CYCLE_DETECTION)
	        }
		}
		return null
	}

	private def Collection<KNode> transformReactorNetwork(
		ReactorInstance reactorInstance,
		Map<PortInstance, KPort> parentInputPorts,
		Map<PortInstance, KPort> parentOutputPorts,
		Map<ReactorInstance, KNode> allReactorNodes
	) {
		val nodes = <KNode>newArrayList
		val inputPorts = HashBasedTable.<ReactorInstance, PortInstance, KPort>create
		val outputPorts = HashBasedTable.<ReactorInstance, PortInstance, KPort>create
		val reactionNodes = <ReactionInstance, KNode>newHashMap
        val directConnectionDummyNodes = <KPort, KNode>newHashMap
		val actionDestinations = HashMultimap.<ActionInstance, KPort>create
		val actionSources = HashMultimap.<ActionInstance, KPort>create
		val timerNodes = <TimerInstance, KNode>newHashMap
		val startupNode = createNode
		var startupUsed = false
		val shutdownNode = createNode
		var shutdownUsed = false

		// Transform instances
		for (entry : reactorInstance.children.reverseView.indexed) {
			val child = entry.value
			// Do not render individual reactors in a bank.
			if (child.getBank() === null) {
			    val rNodes = child.createReactorNode(child.getExpansionState?:false, inputPorts, outputPorts, allReactorNodes)
			    rNodes.head.setLayoutOption(CoreOptions.PRIORITY, entry.key)
			    nodes += rNodes
		    }
		}
		
		// Create timers
		for (timer : reactorInstance.timers) {
			val node = createNode().associateWith(timer.definition)
			node.linkInstance(timer)
			nodes += node
			nodes += timer.definition.createUserComments(node)
			timerNodes.put(timer, node)
			
			node.addTimerFigure(timer)
		}

		// Create reactions
		for (reaction : reactorInstance.reactions.reverseView) {
			val idx = reactorInstance.reactions.indexOf(reaction)
			val node = createNode().associateWith(reaction.definition)
			node.linkInstance(reaction)
			nodes += node
			nodes += reaction.definition.createUserComments(node)
			reactionNodes.put(reaction, node)
			
			node.setLayoutOption(CoreOptions.PORT_CONSTRAINTS, PortConstraints.FIXED_SIDE)
			node.setLayoutOption(CoreOptions.PRIORITY, (reactorInstance.reactions.size - idx) * 10 ) // always place with higher priority than reactor nodes
			node.setLayoutOption(LayeredOptions.POSITION, new KVector(0, idx)) // try order reactions vertically if in one layer
			
			node.addReactionFigure(reaction)
		
			// connect input
			var KPort port
			for (TriggerInstance<?> trigger : reaction.triggers) {
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
 
                if (trigger.startup) {
                    createDependencyEdge((trigger.definition as BuiltinTriggerVariable).definition).connect(startupNode, port)
                    startupUsed = true
                } else if (trigger.shutdown) {
                    createDelayEdge((trigger.definition as BuiltinTriggerVariable).definition).connect(shutdownNode, port)
                    shutdownUsed = true
                } else if (trigger instanceof ActionInstance) {
                    actionDestinations.put(trigger, port)
                } else if (trigger instanceof PortInstance) {
                    var KPort src = null
                    if (trigger.parent === reactorInstance) {
                        src = parentInputPorts.get(trigger)
                    } else {
                        src = outputPorts.get(trigger.parent, trigger)
                    }
                    if (src !== null) {
                        createDependencyEdge(trigger.definition).connect(src, port)
                    }
                } else if (trigger instanceof TimerInstance) {
                    val src = timerNodes.get(trigger)
                    if (src !== null) {
                        createDependencyEdge(trigger.definition).connect(src, port)
                    }
                }
			}
			
			// connect dependencies
			//port = null // create new ports
			for (TriggerInstance<?> dep : reaction.sources.filter[!reaction.triggers.contains(it)]) {
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
				if (dep instanceof PortInstance) {
                    var KPort src = null
                    if (dep.parent === reactorInstance) {
                        src = parentInputPorts.get(dep)
                    } else {
                        src = outputPorts.get(dep.parent, dep)
                    }
                    if (src !== null) {
                        createDependencyEdge(dep).connect(src, port)
                    }
                }
			}
			
			// connect outputs
			port = null // create new ports
			for (TriggerInstance<?> effect : reaction.effects?:emptyList) {
			    // Skip this effect if it is contained in a bank with index other than 0.
			    if (!(effect instanceof PortInstance) 
			        || (effect.parent.bankIndex <= 0)
			    ) {
                    port = if (REACTIONS_USE_HYPEREDGES.booleanValue && port !== null) {
                        port
                    } else {
                        node.addInvisiblePort() => [
                            setLayoutOption(CoreOptions.PORT_SIDE, PortSide.EAST)
                        ]
                    }
                    if (effect instanceof ActionInstance) {
                        actionSources.put(effect, port)
                    } else if (effect instanceof PortInstance) {
                        var KPort dst = null
                        if (effect.isOutput) {
                            dst = parentOutputPorts.get(effect)
                        } else {
                            dst = inputPorts.get(effect.parent, effect)
                        }
                        if (dst !== null) {
                            createDependencyEdge(effect).connect(port, dst)
                        }
                    }
                }
			}
		}
		
		// Connect actions
		val actions = newHashSet
		actions += actionSources.keySet
		actions += actionDestinations.keySet
		for (ActionInstance action : actions) {
			val node = createNode().associateWith(action.definition)
			node.linkInstance(action)
			nodes += node
			nodes += action.definition.createUserComments(node)
			
			node.setLayoutOption(CoreOptions.PORT_CONSTRAINTS, PortConstraints.FIXED_SIDE)
			
			val ports = node.addActionFigureAndPorts(action.isPhysical ? "P" : "L")
			// TODO handle variables?
			if (action.minDelay !== null && action.minDelay !== ActionInstance.DEFAULT_MIN_DELAY) {
				node.addOutsideBottomCenteredNodeLabel('''min delay: «action.minDelay.toString»''', 7)
			}
			// TODO default value?
			if (action.definition.minSpacing !== null) {
                node.addOutsideBottomCenteredNodeLabel('''min spacing: «action.minSpacing.toString»''', 7)
            }
            if (!action.definition.policy.isNullOrEmpty) {
                node.addOutsideBottomCenteredNodeLabel('''policy: «action.policy»''', 7)
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

		// Transform connections.
		// The connections data structure maps connections to their connections as they appear
        // in a visualization of the program. For each connection, there is map
        // from source ports (single ports and multiports) on the left side of the
        // connection to a set of destination ports (single ports and multiports)
        // on the right side of the connection. The ports contained by the multiports
        // are not represented.		
		for (Connection connection : reactorInstance.connections.keySet) {
		    // TODO check banks
		    val connections = reactorInstance.connections.get(connection);
		    for (leftPort : connections.keySet) {
                val rightPorts = connections.get(leftPort);
                for (rightPort : rightPorts) {
                    val source = if (leftPort.parent == reactorInstance) {
                            parentInputPorts.get(leftPort)
                        } else {
                            outputPorts.get(leftPort.parent, leftPort)
                        }
                    val target = if (rightPort.parent == reactorInstance) {
                            parentOutputPorts.get(rightPort)
                        } else {
                            inputPorts.get(rightPort.parent, rightPort)
                        }
                    val edge = createIODependencyEdge(connection, leftPort.isMultiport() || rightPort.isMultiport())
                    if (connection.delay !== null) {
                        edge.addCenterEdgeLabel(connection.delay.toText) => [
                            associateWith(connection.delay)
                            if (connection.physical) {
                                applyOnEdgePysicalDelayStyle(
                                    reactorInstance.mainOrFederated ? Colors.WHITE : Colors.GRAY_95)
                            } else {
                                applyOnEdgeDelayStyle()
                            }
                        ]
                    } else if (connection.physical) {
                        edge.addCenterEdgeLabel("---").applyOnEdgePysicalStyle(
                            reactorInstance.mainOrFederated ? Colors.WHITE : Colors.GRAY_95)
                    }
                    if (source !== null && target !== null) {
                        // check for inside loop (direct in -> out connection with delay)
                        if (parentInputPorts.values.contains(source) && parentOutputPorts.values.contains(target)) {
                            // edge.setLayoutOption(CoreOptions.INSIDE_SELF_LOOPS_YO, true) // Does not work as expected
                            // Introduce dummy node to enable direct connection (that is also hidden when collapsed)
                            var dummy = createNode()
                            if (directConnectionDummyNodes.containsKey(target)) {
                                dummy = directConnectionDummyNodes.get(target)
                            } else {
                                nodes += dummy
                                directConnectionDummyNodes.put(target, dummy)

                                dummy.addInvisibleContainerRendering()
                                dummy.setNodeSize(0, 0)

                                val extraEdge = createIODependencyEdge(null,
                                    leftPort.isMultiport() || rightPort.isMultiport())
                                extraEdge.connect(dummy, target)
                            }
                            edge.connect(source, dummy)
                        } else {
                            edge.connect(source, target)
                        }
                    }
                }
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
		if (reactorInstance.reactions.size > 1) {
			var prevNode = reactionNodes.get(reactorInstance.reactions.head)
			for (node : reactorInstance.reactions.drop(1).map[reactionNodes.get(it)]) {
				val edge = createOrderEdge()
				edge.source = prevNode
				edge.target = node
				edge.setProperty(CoreOptions.NO_LAYOUT, true)
				
				// Do not remove them, as they are needed for cycle detection
				edge.KRendering.invisible = !SHOW_REACTION_ORDER_EDGES.booleanValue
				edge.KRendering.invisible.propagateToChildren = true
				// TODO this does not work work with incremental update (https://github.com/kieler/KLighD/issues/37)
				// if (!SHOW_REACTION_ORDER_EDGES.booleanValue) edge.initiallyHide()
				
				prevNode = node
			}
		}
		
		return nodes
	}
	
	private def String createReactorLabel(ReactorInstance reactorInstance) {
        val b = new StringBuilder
        if (SHOW_INSTANCE_NAMES.booleanValue && !reactorInstance.isRoot) {
            if (!reactorInstance.mainOrFederated) {
                b.append(reactorInstance.name).append(" : ")
            }
        }
        if (reactorInstance.mainOrFederated) {
            b.append(FileConfig.nameWithoutExtension(reactorInstance.reactorDefinition.eResource))
        } else if (reactorInstance.reactorDefinition === null) {
            // There is an error in the graph.
            b.append("<Unresolved Reactor>")
        } else {
            b.append(reactorInstance.reactorDefinition.name)
        }
        if (REACTOR_PARAMETER_MODE.objectValue === ReactorParameterDisplayModes.TITLE) {
            // If the reactor is a bank, then obtain the details from the first
            // element of the bank rather than the bank itself.
            val instance = if (reactorInstance.bankSize > 0) {
                reactorInstance.bankMembers.get(0)
            } else {
                reactorInstance
            }
            if (instance.parameters.empty) {
                b.append("()")
            } else {
                b.append(instance.parameters.join("(", ", ", ")") [
                    createParameterLabel(false)
                ])
            }
        }
        return b.toString()
    }
	
	private def addParameterList(KContainerRendering container, List<ParameterInstance> parameters) {
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
	
	private def String createParameterLabel(ParameterInstance param, boolean bullet) {
		val b = new StringBuilder
		if (bullet) {
			b.append("\u2022 ")
		}
		b.append(param.name)
		val t = param.type.toText
		if (!t.nullOrEmpty) {
			b.append(":").append(t)
		}
		if (!param.init.nullOrEmpty) {
		    b.append("(").append(param.init.join(", ", [it.toText])).append(")")
		}
		return b.toString()
	}
	
	private def createDelayEdge(Object associate) {
		return createEdge => [
			associateWith(associate)
			addPolyline() => [
                boldLineSelectionStyle()
                addJunctionPointDecorator()
				if (USE_ALTERNATIVE_DASH_PATTERN.booleanValue) {
					lineStyle = LineStyle.CUSTOM
					lineStyle.dashPattern += ALTERNATIVE_DASH_PATTERN
				} else {
					lineStyle = LineStyle.DASH
				}
			]
		]
	}
	
	private def createIODependencyEdge(Object associate, boolean multiport) {
		return createEdge => [
			if (associate !== null) {
				associateWith(associate)
			}
			addPolyline() => [
                boldLineSelectionStyle()
			    addJunctionPointDecorator()
				if (multiport) {
                    // Render multiport connections and bank connections in bold.
                    lineWidth = 2.2f
                    lineCap = LineCap.CAP_SQUARE
                    // Adjust junction point size
                    setJunctionPointDecorator(it.junctionPointRendering, 6, 6)
				}
			]
		]
	}
	
	private def createDependencyEdge(Object associate) {
		return createEdge => [
			if (associate !== null) {
				associateWith(associate)
			}
			addPolyline() => [
                boldLineSelectionStyle()
                addJunctionPointDecorator()
				if (USE_ALTERNATIVE_DASH_PATTERN.booleanValue) {
					lineStyle = LineStyle.CUSTOM
					lineStyle.dashPattern += ALTERNATIVE_DASH_PATTERN
				} else {
					lineStyle = LineStyle.DASH
				}
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
				//addFixedTailArrowDecorator() // Fix for bug: https://github.com/kieler/KLighD/issues/38
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
	private def addIOPort(KNode node, PortInstance lfPort, boolean input, boolean multiport, boolean bank) {
		val port = createPort
		node.ports += port
		
		port.associateWith(lfPort.definition)
		port.linkInstance(lfPort)
		port.setPortSize(6, 6)
		
		if (input) {
            // multiports are smaller by an offset at the right, hence compensate in inputs
            val offset = multiport ? -3.4 : -3.3
			port.setLayoutOption(CoreOptions.PORT_SIDE, PortSide.WEST)
			port.setLayoutOption(CoreOptions.PORT_BORDER_OFFSET, offset)
		} else {
		    var offset = (multiport ? -2.6 : -3.3) // multiports are smaller
		    offset = bank ? offset - LinguaFrancaShapeExtensions.BANK_FIGURE_X_OFFSET_SUM : offset // compensate bank figure width
			port.setLayoutOption(CoreOptions.PORT_SIDE, PortSide.EAST)
			port.setLayoutOption(CoreOptions.PORT_BORDER_OFFSET, offset)
		}
		
		if (bank && !node.getProperty(REACTOR_HAS_BANK_PORT_OFFSET)) {// compensate bank figure height
		    // https://github.com/eclipse/elk/issues/693
		    node.getPortMarginsInitIfAbsent().add(new ElkMargin(0, 0, LinguaFrancaShapeExtensions.BANK_FIGURE_Y_OFFSET_SUM, 0))
		    node.setProperty(REACTOR_HAS_BANK_PORT_OFFSET, true) // only once
		}
		
		port.addTrianglePort(multiport)
		
		var label = lfPort.name
		if (!SHOW_PORT_NAMES.booleanValue) {
		    label = ""
		}
		if (SHOW_MULTIPORT_WIDTH.booleanValue) {
            if (lfPort.isMultiport) {
                // TODO Fix unresolvable references in ReactorInstance
                label += "[" + lfPort.width + "]"
            }
		}
		port.addOutsidePortLabel(label, 8).associateWith(lfPort.definition)

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
	
	private def Iterable<KNode> createUserComments(EObject element, KNode targetNode) {
		if (SHOW_USER_LABELS.booleanValue) {
			val commentText = ASTUtils.findAnnotationInComments(element, "@label")
			
			if (!commentText.nullOrEmpty) {
				val comment = createNode()
		        comment.setLayoutOption(CoreOptions.COMMENT_BOX, true)
		        comment.addCommentFigure(commentText) => [
		        	commentStyle()
		        ]
		        
		        // connect
		        createEdge() => [
		        	source = comment
		        	target = targetNode
		        	addCommentPolyline().commentStyle()
		        ]  
		        
		        return #[comment]
			}
		}
		return #[]
	}

}
