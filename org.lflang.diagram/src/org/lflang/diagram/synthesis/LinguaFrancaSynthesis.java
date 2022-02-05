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
package org.lflang.diagram.synthesis;

import com.google.common.base.Objects;
import com.google.common.collect.HashBasedTable;
import com.google.common.collect.HashMultimap;
import com.google.common.collect.Iterables;
import com.google.common.collect.Table;
import de.cau.cs.kieler.klighd.DisplayedActionData;
import de.cau.cs.kieler.klighd.SynthesisOption;
import de.cau.cs.kieler.klighd.kgraph.EMapPropertyHolder;
import de.cau.cs.kieler.klighd.kgraph.KEdge;
import de.cau.cs.kieler.klighd.kgraph.KGraphElement;
import de.cau.cs.kieler.klighd.kgraph.KLabel;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.kgraph.KPort;
import de.cau.cs.kieler.klighd.krendering.Colors;
import de.cau.cs.kieler.klighd.krendering.HorizontalAlignment;
import de.cau.cs.kieler.klighd.krendering.KContainerRendering;
import de.cau.cs.kieler.klighd.krendering.KInvisibility;
import de.cau.cs.kieler.klighd.krendering.KPolyline;
import de.cau.cs.kieler.klighd.krendering.KRectangle;
import de.cau.cs.kieler.klighd.krendering.KRendering;
import de.cau.cs.kieler.klighd.krendering.KRoundedRectangle;
import de.cau.cs.kieler.klighd.krendering.KStyle;
import de.cau.cs.kieler.klighd.krendering.KText;
import de.cau.cs.kieler.klighd.krendering.LineCap;
import de.cau.cs.kieler.klighd.krendering.LineStyle;
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared;
import de.cau.cs.kieler.klighd.krendering.extensions.KContainerRenderingExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KEdgeExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KLabelExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KNodeExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KPolylineExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KPortExtensions;
import de.cau.cs.kieler.klighd.krendering.extensions.KRenderingExtensions;
import de.cau.cs.kieler.klighd.syntheses.AbstractDiagramSynthesis;
import de.cau.cs.kieler.klighd.util.KlighdProperties;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import javax.inject.Inject;
import org.eclipse.elk.alg.layered.options.EdgeStraighteningStrategy;
import org.eclipse.elk.alg.layered.options.FixedAlignment;
import org.eclipse.elk.alg.layered.options.LayerConstraint;
import org.eclipse.elk.alg.layered.options.LayeredOptions;
import org.eclipse.elk.core.math.ElkMargin;
import org.eclipse.elk.core.math.ElkPadding;
import org.eclipse.elk.core.math.KVector;
import org.eclipse.elk.core.options.BoxLayouterOptions;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.elk.core.options.Direction;
import org.eclipse.elk.core.options.PortConstraints;
import org.eclipse.elk.core.options.PortSide;
import org.eclipse.elk.core.options.SizeConstraint;
import org.eclipse.elk.graph.properties.Property;
import org.eclipse.emf.common.util.EList;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.xtend2.lib.StringConcatenation;
import org.eclipse.xtext.xbase.lib.CollectionLiterals;
import org.eclipse.xtext.xbase.lib.Conversions;
import org.eclipse.xtext.xbase.lib.Exceptions;
import org.eclipse.xtext.xbase.lib.Extension;
import org.eclipse.xtext.xbase.lib.Functions.Function1;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.ListExtensions;
import org.eclipse.xtext.xbase.lib.ObjectExtensions;
import org.eclipse.xtext.xbase.lib.Pair;
import org.eclipse.xtext.xbase.lib.Procedures.Procedure1;
import org.eclipse.xtext.xbase.lib.StringExtensions;
import org.lflang.ASTUtils;
import org.lflang.FileConfig;
import org.lflang.diagram.synthesis.action.CollapseAllReactorsAction;
import org.lflang.diagram.synthesis.action.ExpandAllReactorsAction;
import org.lflang.diagram.synthesis.action.FilterCycleAction;
import org.lflang.diagram.synthesis.action.MemorizingExpandCollapseAction;
import org.lflang.diagram.synthesis.action.ShowCycleAction;
import org.lflang.diagram.synthesis.styles.LinguaFrancaShapeExtensions;
import org.lflang.diagram.synthesis.styles.LinguaFrancaStyleExtensions;
import org.lflang.diagram.synthesis.styles.ReactorFigureComponents;
import org.lflang.diagram.synthesis.util.CycleVisualization;
import org.lflang.diagram.synthesis.util.InterfaceDependenciesVisualization;
import org.lflang.diagram.synthesis.util.NamedInstanceUtil;
import org.lflang.diagram.synthesis.util.ReactorIcons;
import org.lflang.diagram.synthesis.util.SynthesisErrorReporter;
import org.lflang.diagram.synthesis.util.UtilityExtensions;
import org.lflang.generator.ActionInstance;
import org.lflang.generator.ParameterInstance;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
import org.lflang.generator.TimerInstance;
import org.lflang.generator.TriggerInstance;
import org.lflang.lf.Connection;
import org.lflang.lf.Delay;
import org.lflang.lf.Model;
import org.lflang.lf.Reaction;
import org.lflang.lf.Reactor;
import org.lflang.lf.Value;
import org.lflang.lf.Variable;

/**
 * Diagram synthesis for Lingua Franca programs.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
@ViewSynthesisShared
class LinguaFrancaSynthesis extends AbstractDiagramSynthesis<Model> {
    @Inject @Extension private KNodeExtensions _kNodeExtensions;
    @Inject @Extension private KEdgeExtensions _kEdgeExtensions;
    @Inject @Extension private KPortExtensions _kPortExtensions;
    @Inject @Extension private KLabelExtensions _kLabelExtensions;
    @Inject @Extension private KRenderingExtensions _kRenderingExtensions;
    @Inject @Extension private KContainerRenderingExtensions _kContainerRenderingExtensions;
    @Inject @Extension private KPolylineExtensions _kPolylineExtensions;
    @Inject @Extension private LinguaFrancaStyleExtensions _linguaFrancaStyleExtensions;
    @Inject @Extension private LinguaFrancaShapeExtensions _linguaFrancaShapeExtensions;
    @Inject @Extension private UtilityExtensions _utilityExtensions;
    @Inject @Extension private CycleVisualization _cycleVisualization;
    @Inject @Extension private InterfaceDependenciesVisualization _interfaceDependenciesVisualization;
    @Inject @Extension private FilterCycleAction _filterCycleAction;
    @Inject @Extension private ReactorIcons _reactorIcons;
	
	// -------------------------------------------------------------------------
	
    public static final String ID = "org.lflang.diagram.synthesis.LinguaFrancaSynthesis";

	// -- INTERNAL --
    public static final Property<Boolean> REACTOR_RECURSIVE_INSTANTIATION = new Property<>("org.lflang.linguafranca.diagram.synthesis.reactor.recursive.instantiation", false);
    public static final Property<Boolean> REACTOR_HAS_BANK_PORT_OFFSET = new Property<>("org.lflang.linguafranca.diagram.synthesis.reactor.bank.offset", false);
    public static final Property<Boolean> REACTOR_INPUT = new Property<>("org.lflang.linguafranca.diagram.synthesis.reactor.input", false);
    public static final Property<Boolean> REACTOR_OUTPUT = new Property<>("org.lflang.linguafranca.diagram.synthesis.reactor.output", false);
    
	// -- STYLE --	
    public static final List<Float> ALTERNATIVE_DASH_PATTERN = List.of(3.0f);
	
	// -- TEXT --
    public static final String TEXT_ERROR_RECURSIVE = "Recursive reactor instantiation!";
    public static final String TEXT_ERROR_CONTAINS_RECURSION = "Reactor contains recursive instantiation!";
    public static final String TEXT_ERROR_CONTAINS_CYCLE = "Reactor contains cyclic dependencies!";
    public static final String TEXT_ERROR_CYCLE_DETECTION = "Dependency cycle detection failed.\nCould not detect dependency cycles due to unexpected graph structure.";
    public static final String TEXT_ERROR_CYCLE_BTN_SHOW = "Show Cycle";
    public static final String TEXT_ERROR_CYCLE_BTN_FILTER = "Filter Cycle";
    public static final String TEXT_ERROR_CYCLE_BTN_UNFILTER = "Remove Cycle Filter";
    public static final String TEXT_NO_MAIN_REACTOR = "No Main Reactor";
    public static final String TEXT_REACTOR_NULL = "Reactor is null";
    public static final String TEXT_HIDE_ACTION = "[Hide]";
    public static final String TEXT_SHOW_ACTION = "[Details]";
	
	// -------------------------------------------------------------------------
	
	/** Synthesis category */
    public static final SynthesisOption APPEARANCE = SynthesisOption.createCategory("Appearance", true);
    public static final SynthesisOption EXPERIMENTAL = SynthesisOption.createCategory("Experimental", true);
    
	/** Synthesis options */
    public static final SynthesisOption SHOW_ALL_REACTORS = SynthesisOption.createCheckOption("All Reactors", Boolean.valueOf(false));
    public static final SynthesisOption CYCLE_DETECTION = SynthesisOption.createCheckOption("Dependency Cycle Detection", Boolean.valueOf(true));
    
    public static final SynthesisOption SHOW_USER_LABELS = SynthesisOption.createCheckOption("User Labels (@label in JavaDoc)", Boolean.valueOf(true)).setCategory(LinguaFrancaSynthesis.APPEARANCE);
    public static final SynthesisOption SHOW_HYPERLINKS = SynthesisOption.createCheckOption("Expand/Collapse Hyperlinks", Boolean.valueOf(false)).setCategory(LinguaFrancaSynthesis.APPEARANCE);
    public static final SynthesisOption REACTIONS_USE_HYPEREDGES = SynthesisOption.createCheckOption("Bundled Dependencies", Boolean.valueOf(false)).setCategory(LinguaFrancaSynthesis.APPEARANCE);
    public static final SynthesisOption USE_ALTERNATIVE_DASH_PATTERN = SynthesisOption.createCheckOption("Alternative Dependency Line Style", Boolean.valueOf(false)).setCategory(LinguaFrancaSynthesis.APPEARANCE);
    public static final SynthesisOption SHOW_PORT_NAMES = SynthesisOption.createCheckOption("Port names", Boolean.valueOf(true)).setCategory(LinguaFrancaSynthesis.APPEARANCE);
    public static final SynthesisOption SHOW_MULTIPORT_WIDTH = SynthesisOption.createCheckOption("Multiport Widths", Boolean.valueOf(false)).setCategory(LinguaFrancaSynthesis.APPEARANCE);
    public static final SynthesisOption SHOW_REACTION_CODE = SynthesisOption.createCheckOption("Reaction Code", Boolean.valueOf(false)).setCategory(LinguaFrancaSynthesis.APPEARANCE);
    public static final SynthesisOption SHOW_REACTION_LEVEL = SynthesisOption.createCheckOption("Reaction Level", Boolean.valueOf(false)).setCategory(LinguaFrancaSynthesis.APPEARANCE);
    public static final SynthesisOption SHOW_REACTION_ORDER_EDGES = SynthesisOption.createCheckOption("Reaction Order Edges", Boolean.valueOf(false)).setCategory(LinguaFrancaSynthesis.APPEARANCE);
    public static final SynthesisOption SHOW_REACTOR_HOST = SynthesisOption.createCheckOption("Reactor Host Addresses", Boolean.valueOf(true)).setCategory(LinguaFrancaSynthesis.APPEARANCE);
    public static final SynthesisOption SHOW_INSTANCE_NAMES = SynthesisOption.createCheckOption("Reactor Instance Names", Boolean.valueOf(false)).setCategory(LinguaFrancaSynthesis.APPEARANCE);
    public static final SynthesisOption REACTOR_PARAMETER_MODE = SynthesisOption.createChoiceOption("Reactor Parameters", ((List<?>)Conversions.doWrapArray(ReactorParameterDisplayModes.values())), ReactorParameterDisplayModes.NONE).setCategory(LinguaFrancaSynthesis.APPEARANCE);
    public static final SynthesisOption REACTOR_PARAMETER_TABLE_COLS = SynthesisOption.<Integer>createRangeOption("Reactor Parameter Table Columns", Integer.valueOf(1), Integer.valueOf(10), Integer.valueOf(1)).setCategory(LinguaFrancaSynthesis.APPEARANCE);
    
    /** Synthesis actions */
    public static final DisplayedActionData COLLAPSE_ALL = DisplayedActionData.create(CollapseAllReactorsAction.ID, "Hide all Details");
    public static final DisplayedActionData EXPAND_ALL = DisplayedActionData.create(ExpandAllReactorsAction.ID, "Show all Details");
    
    @Override
    public List<SynthesisOption> getDisplayedSynthesisOptions() {
		return List.of(
			SHOW_ALL_REACTORS,
			MemorizingExpandCollapseAction.MEMORIZE_EXPANSION_STATES,
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
		);
	}
	
    @Override
    public List<DisplayedActionData> getDisplayedActions() {
        return List.of(COLLAPSE_ALL, EXPAND_ALL);
    }
	
	// -------------------------------------------------------------------------
	
    @Override
    public KNode transform(final Model model) {
        KNode rootNode = _kNodeExtensions.createNode();

		try {
			// Find main
		    Reactor main = IterableExtensions.findFirst(model.getReactors(), _utilityExtensions::isMainOrFederated);
			if (main != null) {
			    ReactorInstance reactorInstance = new ReactorInstance(main, new SynthesisErrorReporter());
			    rootNode.getChildren().addAll(createReactorNode(reactorInstance, true, null, null, new HashMap<>()));
			} else {
			    KNode messageNode = _kNodeExtensions.createNode();
			    _linguaFrancaShapeExtensions.addErrorMessage(messageNode, TEXT_NO_MAIN_REACTOR, null);
			    rootNode.getChildren().add(messageNode);
			}
			
			// Show all reactors
			if (main == null || getBooleanValue(LinguaFrancaSynthesis.SHOW_ALL_REACTORS)) {
			    List<KNode> reactorNodes = new ArrayList<>();
				for (Reactor reactor : model.getReactors()) {
				    if (reactor == main) continue;
				    ReactorInstance reactorInstance = new ReactorInstance(reactor, new SynthesisErrorReporter(), new HashSet<>());
				    reactorNodes.addAll(createReactorNode(reactorInstance, main == null, 
				            HashBasedTable.<ReactorInstance, PortInstance, KPort>create(), 
				            HashBasedTable.<ReactorInstance, PortInstance, KPort>create(), 
				            new HashMap<>()));
					}
				if (!reactorNodes.isEmpty()) {
					// To allow ordering, we need box layout but we also need layered layout for ports thus wrap all node
					// TODO use rect packing in the future
					reactorNodes.add(0, IterableExtensions.head(rootNode.getChildren()));
					
					int index = 0;
					for (KNode node : reactorNodes) {
					    if (node.getProperty(CoreOptions.COMMENT_BOX)) continue;
					    KNode child = _kNodeExtensions.createNode();
					    child.getChildren().add(node);
					    // Add comment nodes
					    for (KEdge edge : node.getIncomingEdges()) {
					        if (!edge.getSource().getProperty(CoreOptions.COMMENT_BOX)) continue;
					        child.getChildren().add(edge.getSource());
					    }
					    _kRenderingExtensions.addInvisibleContainerRendering(child);
					    setLayoutOption(child, CoreOptions.ALGORITHM, LayeredOptions.ALGORITHM_ID);
					    setLayoutOption(child, CoreOptions.PADDING, new ElkPadding(0));
                        setLayoutOption(child, CoreOptions.PRIORITY, reactorNodes.size() - index); // Order!
					    rootNode.getChildren().add(child);
						index++;
					}
					
					setLayoutOption(rootNode, CoreOptions.ALGORITHM, BoxLayouterOptions.ALGORITHM_ID);
			        setLayoutOption(rootNode, CoreOptions.SPACING_NODE_NODE, 25.0);
				}
			}
		} catch (Exception e) {
			e.printStackTrace();
			
			KNode messageNode = _kNodeExtensions.createNode();
			_linguaFrancaShapeExtensions.addErrorMessage(messageNode, "Error in Diagram Synthesis", 
			        e.getClass().getSimpleName() + " occurred. Could not create diagram.");
			rootNode.getChildren().add(messageNode);
		}

		return rootNode;
	}
	
	private Collection<KNode> createReactorNode(
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
            val instance = reactorInstance
            
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
	
	private KNode configureReactorNodeLayout(KNode node) {
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
	
	private KNode detectAndAnnotateCycles(KNode node, ReactorInstance reactorInstance, Map<ReactorInstance, KNode> allReactorNodes) {
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

	private Collection<KNode> transformReactorNetwork(
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
		    val rNodes = child.createReactorNode(child.getExpansionState?:false, inputPorts, outputPorts, allReactorNodes)
		    rNodes.head.setLayoutOption(CoreOptions.PRIORITY, entry.key)
		    nodes += rNodes
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
		// First, collect all the source ports.
		val sourcePorts = new LinkedList<PortInstance>(reactorInstance.inputs);
		for (child : reactorInstance.children) {
		    sourcePorts.addAll(child.outputs);
		}

		for (leftPort : sourcePorts) {
            val source = if (leftPort.parent == reactorInstance) {
                    parentInputPorts.get(leftPort)
                } else {
                    outputPorts.get(leftPort.parent, leftPort)
                }
            for (sendRange : leftPort.dependentPorts) {
                for (rightRange : sendRange.destinations) {
                    val rightPort = rightRange.instance;
                    val target = if (rightPort.parent == reactorInstance) {
                            parentOutputPorts.get(rightPort)
                        } else {
                            inputPorts.get(rightPort.parent, rightPort)
                        }
                    // There should be a connection, but skip if not.
                    val connection = sendRange.connection;
                    if (connection !== null) {
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
	
	private String createReactorLabel(ReactorInstance reactorInstance) {
        val b = new StringBuilder
        if (SHOW_INSTANCE_NAMES.booleanValue && !reactorInstance.isRoot) {
            if (!reactorInstance.mainOrFederated) {
                b.append(reactorInstance.name).append(" : ")
            }
        }
        if (reactorInstance.mainOrFederated) {
            b.append(FileConfig.nameWithoutExtension(reactorInstance.reactorDeclaration.eResource))
        } else if (reactorInstance.reactorDeclaration === null) {
            // There is an error in the graph.
            b.append("<Unresolved Reactor>")
        } else {
            b.append(reactorInstance.reactorDeclaration.name)
        }
        if (REACTOR_PARAMETER_MODE.objectValue === ReactorParameterDisplayModes.TITLE) {
            if (reactorInstance.parameters.empty) {
                b.append("()")
            } else {
                b.append(reactorInstance.parameters.join("(", ", ", ")") [
                    createParameterLabel(false)
                ])
            }
        }
        return b.toString()
    }
	
	private void addParameterList(KContainerRendering container, List<ParameterInstance> parameters) {
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
	
	private String createParameterLabel(ParameterInstance param, boolean bullet) {
		val b = new StringBuilder
		if (bullet) {
			b.append("\u2022 ")
		}
		b.append(param.name)
		val t = param.type.toText
		if (!t.nullOrEmpty) {
			b.append(":").append(t)
		}
		if (!param.getInitialValue.nullOrEmpty) {
		    b.append("(").append(param.getInitialValue.join(", ", [it.toText])).append(")")
		}
		return b.toString()
	}
	
	private KEdge createDelayEdge(Object associate) {
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
	
	private KEdge createIODependencyEdge(Object associate, boolean multiport) {
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
	
	private KEdge createDependencyEdge(Object associate) {
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
	
	private KEdge createOrderEdge() {
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
	
	private KEdge connect(KEdge edge, KNode src, KNode dst) {
		edge.source = src
		edge.target = dst
		
		return edge
	}
	private KEdge connect(KEdge edge, KNode src, KPort dst) {
		edge.source = src
		edge.targetPort = dst
		edge.target = dst?.node
		
		return edge
	}
	private KEdge connect(KEdge edge, KPort src, KNode dst) {
		edge.sourcePort = src
		edge.source = src?.node
		edge.target = dst
		
		return edge
	}
	private KEdge connect(KEdge edge, KPort src, KPort dst) {
		edge.sourcePort = src
		edge.source = src?.node
		edge.targetPort = dst
		edge.target = dst?.node
		
		return edge
	}
	
	/**
	 * Translate an input/output into a port.
	 */
	private KPort addIOPort(KNode node, PortInstance lfPort, boolean input, boolean multiport, boolean bank) {
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
                label += (lfPort.width >= 0)? 
                        "[" + lfPort.width + "]"
                        : "[?]"
            }
		}
		port.addOutsidePortLabel(label, 8).associateWith(lfPort.definition)

		return port
	}

	private KPort addInvisiblePort(KNode node) {
		val port = createPort
		node.ports += port
		
		port.setSize(0, 0) // invisible

		return port
	}
	
	private KNode addErrorComment(KNode node, String message) {
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
	
	private Iterable<KNode> createUserComments(EObject element, KNode targetNode) {
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
