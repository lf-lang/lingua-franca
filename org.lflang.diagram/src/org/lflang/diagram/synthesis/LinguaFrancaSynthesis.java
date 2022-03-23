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

import com.google.common.collect.HashBasedTable;
import com.google.common.collect.HashMultimap;
import com.google.common.collect.Iterables;
import com.google.common.collect.Multimap;
import com.google.common.collect.Table;
import de.cau.cs.kieler.klighd.DisplayedActionData;
import de.cau.cs.kieler.klighd.SynthesisOption;
import de.cau.cs.kieler.klighd.kgraph.KEdge;
import de.cau.cs.kieler.klighd.kgraph.KLabel;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.kgraph.KPort;
import de.cau.cs.kieler.klighd.krendering.Colors;
import de.cau.cs.kieler.klighd.krendering.HorizontalAlignment;
import de.cau.cs.kieler.klighd.krendering.KContainerRendering;
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
import java.util.Collection;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import javax.inject.Inject;
import org.eclipse.elk.alg.layered.options.EdgeStraighteningStrategy;
import org.eclipse.elk.alg.layered.options.FixedAlignment;
import org.eclipse.elk.alg.layered.options.GreedySwitchType;
import org.eclipse.elk.alg.layered.options.LayerConstraint;
import org.eclipse.elk.alg.layered.options.LayeredOptions;
import org.eclipse.elk.alg.layered.options.NodePlacementStrategy;
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
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.eclipse.xtext.xbase.lib.Conversions;
import org.eclipse.xtext.xbase.lib.Exceptions;
import org.eclipse.xtext.xbase.lib.Extension;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.ListExtensions;
import org.eclipse.xtext.xbase.lib.Pair;
import org.eclipse.xtext.xbase.lib.StringExtensions;
import org.lflang.ASTUtils;
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
import org.lflang.diagram.synthesis.util.ModeDiagrams;
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
import org.lflang.lf.Model;
import org.lflang.lf.Reactor;
import org.lflang.util.FileUtil;

/**
 * Diagram synthesis for Lingua Franca programs.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
@ViewSynthesisShared
public class LinguaFrancaSynthesis extends AbstractDiagramSynthesis<Model> {
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
    @Inject @Extension private ModeDiagrams _modeDiagrams;
    
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
    public static final SynthesisOption SHOW_ALL_REACTORS = SynthesisOption.createCheckOption("All Reactors", false);
    public static final SynthesisOption CYCLE_DETECTION = SynthesisOption.createCheckOption("Dependency Cycle Detection", true);
    
    public static final SynthesisOption SHOW_USER_LABELS = SynthesisOption.createCheckOption("User Labels (@label in JavaDoc)", true).setCategory(APPEARANCE);
    public static final SynthesisOption SHOW_HYPERLINKS = SynthesisOption.createCheckOption("Expand/Collapse Hyperlinks", false).setCategory(APPEARANCE);
    public static final SynthesisOption REACTIONS_USE_HYPEREDGES = SynthesisOption.createCheckOption("Bundled Dependencies", false).setCategory(APPEARANCE);
    public static final SynthesisOption USE_ALTERNATIVE_DASH_PATTERN = SynthesisOption.createCheckOption("Alternative Dependency Line Style", false).setCategory(APPEARANCE);
    public static final SynthesisOption SHOW_PORT_NAMES = SynthesisOption.createCheckOption("Port names", true).setCategory(APPEARANCE);
    public static final SynthesisOption SHOW_MULTIPORT_WIDTH = SynthesisOption.createCheckOption("Multiport Widths", false).setCategory(APPEARANCE);
    public static final SynthesisOption SHOW_REACTION_CODE = SynthesisOption.createCheckOption("Reaction Code", false).setCategory(APPEARANCE);
    public static final SynthesisOption SHOW_REACTION_LEVEL = SynthesisOption.createCheckOption("Reaction Level", false).setCategory(APPEARANCE);
    public static final SynthesisOption SHOW_REACTION_ORDER_EDGES = SynthesisOption.createCheckOption("Reaction Order Edges", false).setCategory(APPEARANCE);
    public static final SynthesisOption SHOW_REACTOR_HOST = SynthesisOption.createCheckOption("Reactor Host Addresses", true).setCategory(APPEARANCE);
    public static final SynthesisOption SHOW_INSTANCE_NAMES = SynthesisOption.createCheckOption("Reactor Instance Names", false).setCategory(APPEARANCE);
    public static final SynthesisOption REACTOR_PARAMETER_MODE = SynthesisOption.createChoiceOption("Reactor Parameters", ((List<?>)Conversions.doWrapArray(ReactorParameterDisplayModes.values())), ReactorParameterDisplayModes.NONE).setCategory(APPEARANCE);
    public static final SynthesisOption REACTOR_PARAMETER_TABLE_COLS = SynthesisOption.<Integer>createRangeOption("Reactor Parameter Table Columns", 1, 10, 1).setCategory(APPEARANCE);
    
    /** Synthesis actions */
    public static final DisplayedActionData COLLAPSE_ALL = DisplayedActionData.create(CollapseAllReactorsAction.ID, "Hide all Details");
    public static final DisplayedActionData EXPAND_ALL = DisplayedActionData.create(ExpandAllReactorsAction.ID, "Show all Details");
    
    @Override
    public List<SynthesisOption> getDisplayedSynthesisOptions() {
        return List.of(
            SHOW_ALL_REACTORS,
            MemorizingExpandCollapseAction.MEMORIZE_EXPANSION_STATES,
            CYCLE_DETECTION,
            ModeDiagrams.SHOW_TRANSITION_LABELS,
            ModeDiagrams.INITIALLY_COLLAPSE_MODES,
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
            if (main == null || getBooleanValue(SHOW_ALL_REACTORS)) {
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
        Reactor reactor = reactorInstance.reactorDefinition;
        KNode node = _kNodeExtensions.createNode();
        allReactorNodes.put(reactorInstance, node);
        associateWith(node, reactor);
        _utilityExtensions.setID(node, reactorInstance.uniqueID());
        // save to distinguish nodes associated with the same reactor
        NamedInstanceUtil.linkInstance(node, reactorInstance);
        
        List<KNode> nodes = new ArrayList<>();
        nodes.add(node);
        String label = createReactorLabel(reactorInstance);

        if (reactorInstance.recursive) {
            // Mark this node
            node.setProperty(REACTOR_RECURSIVE_INSTANTIATION, true);
            // Mark root
            allReactorNodes.get(reactorInstance.root()).setProperty(REACTOR_RECURSIVE_INSTANTIATION, true);
        }
        
        if (reactor == null) {
            _linguaFrancaShapeExtensions.addErrorMessage(node, TEXT_REACTOR_NULL, null);
        } else if (reactorInstance.isMainOrFederated()) {
            KRoundedRectangle figure = _linguaFrancaShapeExtensions.addMainReactorFigure(node, reactorInstance, label);
            
            if (getObjectValue(REACTOR_PARAMETER_MODE) == ReactorParameterDisplayModes.TABLE 
                && !reactorInstance.parameters.isEmpty()
            ) {
                KRectangle rectangle = _kContainerRenderingExtensions.addRectangle(figure);
                _kRenderingExtensions.setInvisible(rectangle, true);
                _kRenderingExtensions.to(
                        _kRenderingExtensions.from(
                                _kRenderingExtensions.setGridPlacementData(rectangle), 
                                _kRenderingExtensions.LEFT, 8, 0, 
                                _kRenderingExtensions.TOP, 0, 0), 
                        _kRenderingExtensions.RIGHT, 8, 0, 
                        _kRenderingExtensions.BOTTOM, 4, 0);
                _kRenderingExtensions.setHorizontalAlignment(rectangle, HorizontalAlignment.LEFT);
                addParameterList(rectangle, reactorInstance.parameters);
            }

            if (reactorInstance.recursive) {
                nodes.add(addErrorComment(node, TEXT_ERROR_RECURSIVE));
                _linguaFrancaStyleExtensions.errorStyle(figure);
            } else {
                _kContainerRenderingExtensions.addChildArea(figure);
                node.getChildren().addAll(transformReactorNetwork(reactorInstance, 
                        new HashMap<>(), 
                        new HashMap<>(), 
                        allReactorNodes));
            }
            Iterables.addAll(nodes, createUserComments(reactor, node));
            configureReactorNodeLayout(node);
            
            // Additional layout adjustment for main node
            setLayoutOption(node, CoreOptions.ALGORITHM, LayeredOptions.ALGORITHM_ID);
            setLayoutOption(node, CoreOptions.DIRECTION, Direction.RIGHT);
            setLayoutOption(node, CoreOptions.NODE_SIZE_CONSTRAINTS, EnumSet.of(SizeConstraint.MINIMUM_SIZE));
            setLayoutOption(node, LayeredOptions.NODE_PLACEMENT_BK_FIXED_ALIGNMENT, FixedAlignment.BALANCED);
            setLayoutOption(node, LayeredOptions.NODE_PLACEMENT_BK_EDGE_STRAIGHTENING, EdgeStraighteningStrategy.IMPROVE_STRAIGHTNESS);
            setLayoutOption(node, LayeredOptions.SPACING_EDGE_NODE, LayeredOptions.SPACING_EDGE_NODE.getDefault() * 1.1f);
            setLayoutOption(node, LayeredOptions.SPACING_EDGE_NODE_BETWEEN_LAYERS, LayeredOptions.SPACING_EDGE_NODE_BETWEEN_LAYERS.getDefault() * 1.1f);
            if (!getBooleanValue(SHOW_HYPERLINKS)) {
                setLayoutOption(node, CoreOptions.PADDING, new ElkPadding(-1, 6, 6, 6));
                setLayoutOption(node, LayeredOptions.SPACING_COMPONENT_COMPONENT, LayeredOptions.SPACING_COMPONENT_COMPONENT.getDefault() * 0.5f);
            }
        } else {
            ReactorInstance instance = reactorInstance;
            
            // Expanded Rectangle
            ReactorFigureComponents comps = _linguaFrancaShapeExtensions.addReactorFigure(node, reactorInstance, label);
            comps.getOuter().setProperty(KlighdProperties.EXPANDED_RENDERING, true);
            for (KRendering figure : comps.getFigures()) {
                associateWith(figure, reactor);
                _kRenderingExtensions.addDoubleClickAction(figure, MemorizingExpandCollapseAction.ID);
            }
            _reactorIcons.handleIcon(comps.getReactor(), reactor, false);
            
            if (getBooleanValue(SHOW_HYPERLINKS)) {
                // Collapse button
                KText button = _linguaFrancaShapeExtensions.addTextButton(comps.getReactor(), TEXT_HIDE_ACTION);
                _kRenderingExtensions.to(
                        _kRenderingExtensions.from(
                                _kRenderingExtensions.setGridPlacementData(button), 
                                _kRenderingExtensions.LEFT, 8, 0, 
                                _kRenderingExtensions.TOP, 0, 0), 
                        _kRenderingExtensions.RIGHT, 8, 0, 
                        _kRenderingExtensions.BOTTOM, 0, 0);
                _kRenderingExtensions.addSingleClickAction(button, MemorizingExpandCollapseAction.ID);
                _kRenderingExtensions.addDoubleClickAction(button, MemorizingExpandCollapseAction.ID);
            }
            
            if (getObjectValue(REACTOR_PARAMETER_MODE) == ReactorParameterDisplayModes.TABLE 
                    && !instance.parameters.isEmpty()) {
                KRectangle rectangle = _kContainerRenderingExtensions.addRectangle(comps.getReactor());
                _kRenderingExtensions.setInvisible(rectangle, true);
                if (!getBooleanValue(SHOW_HYPERLINKS)) {
                    _kRenderingExtensions.to(
                            _kRenderingExtensions.from(
                                    _kRenderingExtensions.setGridPlacementData(rectangle), 
                                    _kRenderingExtensions.LEFT, 8, 0, 
                                    _kRenderingExtensions.TOP, 0, 0), 
                            _kRenderingExtensions.RIGHT, 8, 0, 
                            _kRenderingExtensions.BOTTOM, 4, 0);
                } else {
                    _kRenderingExtensions.to(
                            _kRenderingExtensions.from(
                                    _kRenderingExtensions.setGridPlacementData(rectangle), 
                                    _kRenderingExtensions.LEFT, 8, 0, 
                                    _kRenderingExtensions.TOP, 4, 0), 
                            _kRenderingExtensions.RIGHT, 8, 0, 
                            _kRenderingExtensions.BOTTOM, 0, 0);
                }
                _kRenderingExtensions.setHorizontalAlignment(rectangle, HorizontalAlignment.LEFT);
                addParameterList(rectangle, instance.parameters);
            }
                
            if (instance.recursive) {
                comps.getFigures().forEach(_linguaFrancaStyleExtensions::errorStyle);
            } else {
                _kContainerRenderingExtensions.addChildArea(comps.getReactor());
            }

            // Collapse Rectangle
            comps = _linguaFrancaShapeExtensions.addReactorFigure(node, reactorInstance, label);
            comps.getOuter().setProperty(KlighdProperties.COLLAPSED_RENDERING, true);
            for (KRendering figure : comps.getFigures()) {
                associateWith(figure, reactor);
                if (_utilityExtensions.hasContent(instance) && !instance.recursive) {
                    _kRenderingExtensions.addDoubleClickAction(figure, MemorizingExpandCollapseAction.ID);
                }
            }
            _reactorIcons.handleIcon(comps.getReactor(), reactor, true);
            
            if (getBooleanValue(SHOW_HYPERLINKS)) {
                // Expand button
                if (_utilityExtensions.hasContent(instance) && !instance.recursive) {
                    KText button = _linguaFrancaShapeExtensions.addTextButton(comps.getReactor(), TEXT_SHOW_ACTION);
                    _kRenderingExtensions.to(
                            _kRenderingExtensions.from(
                                    _kRenderingExtensions.setGridPlacementData(button), 
                                    _kRenderingExtensions.LEFT, 8, 0, 
                                    _kRenderingExtensions.TOP, 0, 0), 
                            _kRenderingExtensions.RIGHT, 8, 0, 
                            _kRenderingExtensions.BOTTOM, 8, 0);
                    _kRenderingExtensions.addSingleClickAction(button, MemorizingExpandCollapseAction.ID);
                    _kRenderingExtensions.addDoubleClickAction(button, MemorizingExpandCollapseAction.ID);
                }
            }
                
            if (instance.recursive) {
                comps.getFigures().forEach(_linguaFrancaStyleExtensions::errorStyle);
            }
            
            
            // Create ports
            Map<PortInstance, KPort> inputPorts = new HashMap<>();
            Map<PortInstance, KPort> outputPorts = new HashMap<>();
            for (PortInstance input : ListExtensions.reverseView(instance.inputs)) {
                inputPorts.put(input, addIOPort(node, input, true, input.isMultiport(), reactorInstance.isBank()));
            }
            for (PortInstance output : instance.outputs) {
                outputPorts.put(output, addIOPort(node, output, false, output.isMultiport(), reactorInstance.isBank()));
            }
            // Mark ports
            inputPorts.values().forEach(it -> it.setProperty(REACTOR_INPUT, true));
            outputPorts.values().forEach(it -> it.setProperty(REACTOR_OUTPUT, true));

            // Add content
            if (_utilityExtensions.hasContent(instance) && !instance.recursive) {
                node.getChildren().addAll(transformReactorNetwork(instance, inputPorts, outputPorts, allReactorNodes));
            }
            
            // Pass port to given tables
            if (!_utilityExtensions.isRoot(instance)) {
                if (inputPortsReg != null) {
                    for (Map.Entry<PortInstance, KPort> entry : inputPorts.entrySet()) {
                        inputPortsReg.put(instance, entry.getKey(), entry.getValue());
                    }
                }
                if (outputPortsReg != null) {
                    for (Map.Entry<PortInstance, KPort> entry : outputPorts.entrySet()) {
                        outputPortsReg.put(instance, entry.getKey(), entry.getValue());
                    }
                }
            }
            
            if (instance.recursive) {
                setLayoutOption(node, KlighdProperties.EXPAND, false);
                nodes.add(addErrorComment(node, TEXT_ERROR_RECURSIVE));
            } else {
                setLayoutOption(node, KlighdProperties.EXPAND, expandDefault);
                
                // Interface Dependencies
                _interfaceDependenciesVisualization.addInterfaceDependencies(node, expandDefault);
            }
            
            if (!_utilityExtensions.isRoot(instance)) {
                // If all reactors are being shown, then only put the label on
                // the reactor definition, not on its instances. Otherwise,
                // add the annotation now.
                if (!getBooleanValue(SHOW_ALL_REACTORS)) {
                    Iterables.addAll(nodes, createUserComments(reactor, node));
                }
            } else {
                Iterables.addAll(nodes, createUserComments(reactor, node));
            }
            configureReactorNodeLayout(node);
        }

        // Find and annotate cycles
        if (getBooleanValue(CYCLE_DETECTION) && 
                _utilityExtensions.isRoot(reactorInstance)) {
            KNode errNode = detectAndAnnotateCycles(node, reactorInstance, allReactorNodes);
            if (errNode != null) {
                nodes.add(errNode);
            }
        }

        return nodes;
    }
    
    private KNode configureReactorNodeLayout(KNode node) {
        setLayoutOption(node, CoreOptions.NODE_SIZE_CONSTRAINTS, SizeConstraint.minimumSizeWithPorts());
        // Allows to freely shuffle ports on each side
        setLayoutOption(node, CoreOptions.PORT_CONSTRAINTS, PortConstraints.FIXED_SIDE);
        // Balanced placement with straight long edges.
        setLayoutOption(node, LayeredOptions.NODE_PLACEMENT_STRATEGY, NodePlacementStrategy.NETWORK_SIMPLEX);
        // Otherwise nodes are not sorted if they are not connected
        setLayoutOption(node, CoreOptions.SEPARATE_CONNECTED_COMPONENTS, false);
        // Needed to enforce node positions.
        setLayoutOption(node, LayeredOptions.CROSSING_MINIMIZATION_SEMI_INTERACTIVE, true);
        // Costs a little more time but layout is quick, therefore, we can do that.
        setLayoutOption(node, LayeredOptions.THOROUGHNESS, 100);
        setLayoutOption(node, LayeredOptions.CROSSING_MINIMIZATION_GREEDY_SWITCH_TYPE, GreedySwitchType.TWO_SIDED);
        if (!getBooleanValue(SHOW_HYPERLINKS)) {
            setLayoutOption(node, CoreOptions.PADDING, new ElkPadding(2, 6, 6, 6));
            setLayoutOption(node, LayeredOptions.SPACING_NODE_NODE, LayeredOptions.SPACING_NODE_NODE.getDefault() * 0.75f);
            setLayoutOption(node, LayeredOptions.SPACING_NODE_NODE_BETWEEN_LAYERS, LayeredOptions.SPACING_NODE_NODE_BETWEEN_LAYERS.getDefault() * 0.75f);
            setLayoutOption(node, LayeredOptions.SPACING_EDGE_NODE, LayeredOptions.SPACING_EDGE_NODE.getDefault() * 0.75f);
            setLayoutOption(node, LayeredOptions.SPACING_EDGE_NODE_BETWEEN_LAYERS, LayeredOptions.SPACING_EDGE_NODE_BETWEEN_LAYERS.getDefault() * 0.75f);
        }
        return node;
    }
    
    private KNode detectAndAnnotateCycles(KNode node, ReactorInstance reactorInstance, Map<ReactorInstance, KNode> allReactorNodes) {
        if (node.getProperty(REACTOR_RECURSIVE_INSTANTIATION)) {
            _filterCycleAction.resetCycleFiltering(node);
            return addErrorComment(node, TEXT_ERROR_CONTAINS_RECURSION);
        } else { // only detect dependency cycles if not recursive
            try {
                boolean hasCycle = _cycleVisualization.detectAndHighlightCycles(reactorInstance, 
                        allReactorNodes, it -> {
                            if (it instanceof KNode) {
                                List<KRendering> renderings = IterableExtensions.toList(
                                        Iterables.filter(((KNode) it).getData(), KRendering.class));
                                if (renderings.size() == 1) {
                                     _linguaFrancaStyleExtensions.errorStyle(IterableExtensions.head(renderings));
                                } else {
                                    IterableExtensions.filter(renderings, rendering -> {
                                        return rendering.getProperty(KlighdProperties.COLLAPSED_RENDERING);
                                    }).forEach(_linguaFrancaStyleExtensions::errorStyle);
                                }
                            } else if (it instanceof KEdge) {
                                Iterables.filter(((KEdge) it).getData(), 
                                        KRendering.class).forEach(_linguaFrancaStyleExtensions::errorStyle);
                                // TODO initiallyHide does not work with incremental (https://github.com/kieler/KLighD/issues/37)
                                // cycleEgde.initiallyShow() // Show hidden order dependencies
                                _kRenderingExtensions.setInvisible(_kRenderingExtensions.getKRendering(it), false);
                            } else if (it instanceof KPort) {
                                Iterables.filter(((KPort) it).getData(), 
                                        KRendering.class).forEach(_linguaFrancaStyleExtensions::errorStyle);
                                //it.reverseTrianglePort()
                            }
                        });
                
                if (hasCycle) {
                    KNode err = addErrorComment(node, TEXT_ERROR_CONTAINS_CYCLE);
                    
                    // Add to existing figure
                    KRectangle rectangle = _kContainerRenderingExtensions.addRectangle(_kRenderingExtensions.getKContainerRendering(err));
                    _kRenderingExtensions.to(
                            _kRenderingExtensions.from(
                                    _kRenderingExtensions.setGridPlacementData(rectangle), 
                                    _kRenderingExtensions.LEFT, 3, 0, 
                                    _kRenderingExtensions.TOP, (-1), 0),
                            _kRenderingExtensions.RIGHT, 3, 0, 
                            _kRenderingExtensions.BOTTOM, 3, 0);
                    _linguaFrancaStyleExtensions.noSelectionStyle(rectangle);
                    _kRenderingExtensions.setInvisible(rectangle, true);
                    _kContainerRenderingExtensions.setGridPlacement(rectangle, 2);
                    
                    KRectangle subrectangle = _kContainerRenderingExtensions.addRectangle(rectangle);
                    _kRenderingExtensions.to(
                            _kRenderingExtensions.from(
                                    _kRenderingExtensions.setGridPlacementData(subrectangle), 
                                    _kRenderingExtensions.LEFT, 0, 0, 
                                    _kRenderingExtensions.TOP, 0, 0), 
                            _kRenderingExtensions.RIGHT, 2, 0, 
                            _kRenderingExtensions.BOTTOM, 0, 0);
                    _linguaFrancaStyleExtensions.noSelectionStyle(subrectangle);
                    _kRenderingExtensions.addSingleClickAction(subrectangle, ShowCycleAction.ID);
                    
                    KText subrectangleText = _kContainerRenderingExtensions.addText(subrectangle, TEXT_ERROR_CYCLE_BTN_SHOW);
                    // Copy text style
                    List<KStyle> styles = ListExtensions.map(
                            IterableExtensions.head(
                                    _kRenderingExtensions.getKContainerRendering(err).getChildren()).getStyles(), 
                            EcoreUtil::copy);
                    subrectangleText.getStyles().addAll(styles);
                    _kRenderingExtensions.setFontSize(subrectangleText, 5);
                    _kRenderingExtensions.setSurroundingSpace(subrectangleText, 1, 0);
                    _linguaFrancaStyleExtensions.noSelectionStyle(subrectangleText);
                    _kRenderingExtensions.addSingleClickAction(subrectangleText, ShowCycleAction.ID);
                    
                    subrectangle = _kContainerRenderingExtensions.addRectangle(rectangle);
                    _kRenderingExtensions.to(
                            _kRenderingExtensions.from(
                                    _kRenderingExtensions.setGridPlacementData(subrectangle), 
                                    _kRenderingExtensions.LEFT, 0, 0, 
                                    _kRenderingExtensions.TOP, 0, 0), 
                            _kRenderingExtensions.RIGHT, 0, 0, 
                            _kRenderingExtensions.BOTTOM, 0, 0);
                    _linguaFrancaStyleExtensions.noSelectionStyle(subrectangle);
                    _kRenderingExtensions.addSingleClickAction(subrectangle, FilterCycleAction.ID);
                    
                    subrectangleText = _kContainerRenderingExtensions.addText(subrectangle, 
                            _filterCycleAction.isCycleFiltered(node) ? 
                                    TEXT_ERROR_CYCLE_BTN_UNFILTER : TEXT_ERROR_CYCLE_BTN_FILTER);
                    // Copy text style
                    styles = ListExtensions.map(
                            IterableExtensions.head(
                                    _kRenderingExtensions.getKContainerRendering(err).getChildren()).getStyles(), 
                            EcoreUtil::copy);
                    subrectangleText.getStyles().addAll(styles);
                    _kRenderingExtensions.setFontSize(subrectangleText, 5);
                    _kRenderingExtensions.setSurroundingSpace(subrectangleText, 1, 0);
                    _linguaFrancaStyleExtensions.noSelectionStyle(subrectangleText);
                    _kRenderingExtensions.addSingleClickAction(subrectangleText, FilterCycleAction.ID);
                    _filterCycleAction.markCycleFilterText(subrectangleText, err);
                    
                    // if user interactively requested a filtered diagram keep it filtered during updates
                    if (_filterCycleAction.isCycleFiltered(node)) {
                        _filterCycleAction.filterCycle(node);
                    }
                    return err;
                }
            } catch(Exception e) {
                _filterCycleAction.resetCycleFiltering(node);
                e.printStackTrace();
                return addErrorComment(node, TEXT_ERROR_CYCLE_DETECTION);
            }
        }
        return null;
    }

    private Collection<KNode> transformReactorNetwork(
        ReactorInstance reactorInstance,
        Map<PortInstance, KPort> parentInputPorts,
        Map<PortInstance, KPort> parentOutputPorts,
        Map<ReactorInstance, KNode> allReactorNodes
    ) {
        List<KNode> nodes = new ArrayList<>();
        Table<ReactorInstance, PortInstance, KPort> inputPorts = HashBasedTable.create();
        Table<ReactorInstance, PortInstance, KPort> outputPorts = HashBasedTable.create();
        Map<ReactionInstance, KNode> reactionNodes = new HashMap<>();
        Map<KPort, KNode> directConnectionDummyNodes = new HashMap<>();
        Multimap<ActionInstance, KPort> actionDestinations = HashMultimap.create();
        Multimap<ActionInstance, KPort> actionSources = HashMultimap.create();
        Map<TimerInstance, KNode> timerNodes = new HashMap<>();
        KNode startupNode = _kNodeExtensions.createNode();
        boolean startupUsed = false;
        KNode shutdownNode = _kNodeExtensions.createNode();
        boolean shutdownUsed = false;

        // Transform instances
        int index = 0;
        for (ReactorInstance child : ListExtensions.reverseView(reactorInstance.children)) {
            Boolean expansionState = MemorizingExpandCollapseAction.getExpansionState(child);
            Collection<KNode> rNodes = createReactorNode(
                    child, 
                    expansionState != null ? expansionState : false, 
                    inputPorts, 
                    outputPorts, 
                    allReactorNodes);
            setLayoutOption(IterableExtensions.<KNode>head(rNodes), CoreOptions.PRIORITY, index);
            nodes.addAll(rNodes);
            index++;
        }
        
        // Create timers
        for (TimerInstance timer : reactorInstance.timers) {
            KNode node = associateWith(_kNodeExtensions.createNode(), timer.getDefinition());
            NamedInstanceUtil.linkInstance(node, timer);
            _utilityExtensions.setID(node, timer.uniqueID());
            nodes.add(node);
            Iterables.addAll(nodes, createUserComments(timer.getDefinition(), node));
            timerNodes.put(timer, node);
            _linguaFrancaShapeExtensions.addTimerFigure(node, timer);
        }

        // Create reactions
        for (ReactionInstance reaction : ListExtensions.reverseView(reactorInstance.reactions)) {
            int idx = reactorInstance.reactions.indexOf(reaction);
            KNode node = associateWith(_kNodeExtensions.createNode(), reaction.getDefinition());
            NamedInstanceUtil.linkInstance(node, reaction);
            _utilityExtensions.setID(node, reaction.uniqueID());
            nodes.add(node);
            Iterables.addAll(nodes, createUserComments(reaction.getDefinition(), node));
            reactionNodes.put(reaction, node);
            
            setLayoutOption(node, CoreOptions.PORT_CONSTRAINTS, PortConstraints.FIXED_SIDE);
            setLayoutOption(node, CoreOptions.PRIORITY, (reactorInstance.reactions.size() - idx) * 10 ); // always place with higher priority than reactor nodes
            setLayoutOption(node, LayeredOptions.POSITION, new KVector(0, idx)); // try order reactions vertically if in one layer
            
            _linguaFrancaShapeExtensions.addReactionFigure(node, reaction);
        
            // connect input
            KPort port = null;
            for (TriggerInstance<?> trigger : reaction.triggers) {
                // Create new port if there is no previous one or each dependency should have its own one
                if (port == null || !getBooleanValue(REACTIONS_USE_HYPEREDGES)) {
                    port = addInvisiblePort(node);
                    setLayoutOption(port, CoreOptions.PORT_SIDE, PortSide.WEST);
                    
                    int triggersSize = reaction.triggers != null ? reaction.triggers.size() : 0;
                    int sourcesSize  = reaction.sources  != null ? reaction.sources.size()  : 0;
                    if (getBooleanValue(REACTIONS_USE_HYPEREDGES) || triggersSize + sourcesSize == 1) {
                        // manual adjustment disabling automatic one
                        setLayoutOption(port, CoreOptions.PORT_BORDER_OFFSET,
                                (double) -LinguaFrancaShapeExtensions.REACTION_POINTINESS);
                    }
                }
                
                if (trigger.isStartup()) {
                    connect(createDependencyEdge(((TriggerInstance.BuiltinTriggerVariable) trigger.getDefinition()).definition), 
                            startupNode, 
                            port);
                    startupUsed = true;
                } else if (trigger.isShutdown()) {
                    connect(createDelayEdge(((TriggerInstance.BuiltinTriggerVariable) trigger.getDefinition()).definition), 
                            shutdownNode, 
                            port);
                    shutdownUsed = true;
                } else if (trigger instanceof ActionInstance) {
                    actionDestinations.put(((ActionInstance) trigger), port);
                } else if (trigger instanceof PortInstance) {
                    KPort src = null;
                    PortInstance triggerAsPort = (PortInstance) trigger;
                    if (triggerAsPort.getParent() == reactorInstance) {
                        src = parentInputPorts.get(trigger);
                    } else {
                        src = outputPorts.get(triggerAsPort.getParent(), trigger);
                    }
                    if (src != null) {
                        connect(createDependencyEdge(triggerAsPort.getDefinition()), src, port);
                    }
                } else if (trigger instanceof TimerInstance) {
                    KNode src = timerNodes.get(trigger);
                    if (src != null) {
                        connect(createDependencyEdge(trigger.getDefinition()), src, port);
                    }
                }
            }
            
            // connect dependencies
            for (TriggerInstance<?> dep : reaction.sources) {
                if (reaction.triggers.contains(dep)) continue; // skip

                // Create new port if there is no previous one or each dependency should have its own one
                if (port == null || !getBooleanValue(REACTIONS_USE_HYPEREDGES)) {
                    port = addInvisiblePort(node);
                    setLayoutOption(port, CoreOptions.PORT_SIDE, PortSide.WEST);
                    
                    int triggersSize = reaction.triggers != null ? reaction.triggers.size() : 0;
                    int sourcesSize  = reaction.sources  != null ? reaction.sources.size()  : 0;
                    if (getBooleanValue(REACTIONS_USE_HYPEREDGES) || triggersSize + sourcesSize == 1) {
                        // manual adjustment disabling automatic one
                        setLayoutOption(port, CoreOptions.PORT_BORDER_OFFSET, 
                                (double) -LinguaFrancaShapeExtensions.REACTION_POINTINESS);
                    }
                }
                
                if (dep instanceof PortInstance) {
                    KPort src = null;
                    PortInstance depAsPort = (PortInstance) dep;
                    if (dep.getParent() == reactorInstance) {
                        src = parentInputPorts.get(dep);
                    } else {
                        src = outputPorts.get(depAsPort.getParent(), dep);
                    }
                    if (src != null) {
                        connect(createDependencyEdge(dep.getDefinition()), src, port);
                    }
                }
            }
    
            // connect outputs
            port = null; // enforce new ports for outputs
            Set<TriggerInstance<?>> iterSet = reaction.effects != null ? reaction.effects : new HashSet<>();
            for (TriggerInstance<?> effect : iterSet) {
                // Create new port if there is no previous one or each dependency should have its own one
                if (port == null || !getBooleanValue(REACTIONS_USE_HYPEREDGES)) {
                    port = addInvisiblePort(node);
                    setLayoutOption(port, CoreOptions.PORT_SIDE, PortSide.EAST);
                }
                
                if (effect instanceof ActionInstance) {
                    actionSources.put((ActionInstance) effect, port);
                } else if (effect instanceof PortInstance) {
                    KPort dst = null;
                    PortInstance effectAsPort = (PortInstance) effect;
                    if (effectAsPort.isOutput()) {
                        dst = parentOutputPorts.get(effect);
                    } else {
                        dst = inputPorts.get(effectAsPort.getParent(), effect);
                    }
                    if (dst != null) {
                        connect(createDependencyEdge(effect), port, dst);
                    }
                }
            }
        }
            
        // Connect actions
        Set<ActionInstance> actions = new HashSet<>();
        actions.addAll(actionSources.keySet());
        actions.addAll(actionDestinations.keySet());
        
        for (ActionInstance action : actions) {
            KNode node = associateWith(_kNodeExtensions.createNode(), action.getDefinition());
            NamedInstanceUtil.linkInstance(node, action);
            _utilityExtensions.setID(node, action.uniqueID());
            nodes.add(node);
            Iterables.addAll(nodes, createUserComments(action.getDefinition(), node));
            setLayoutOption(node, CoreOptions.PORT_CONSTRAINTS, PortConstraints.FIXED_SIDE);
            Pair<KPort, KPort> ports = _linguaFrancaShapeExtensions.addActionFigureAndPorts(
                    node, 
                    action.isPhysical() ? "P" : "L");
            // TODO handle variables?
            if (action.getMinDelay() != null && action.getMinDelay() != ActionInstance.DEFAULT_MIN_DELAY) {
                _kLabelExtensions.addOutsideBottomCenteredNodeLabel(
                        node, 
                        String.format("min delay: %s", action.getMinDelay().toString()), 
                        7);
            }
           // TODO default value?
            if (action.getDefinition().getMinSpacing() != null) {
                _kLabelExtensions.addOutsideBottomCenteredNodeLabel(node, 
                        String.format("min spacing: %s", action.getMinSpacing().toString()),
                        7);
            }
            if (!StringExtensions.isNullOrEmpty(action.getDefinition().getPolicy())) {
                _kLabelExtensions.addOutsideBottomCenteredNodeLabel(node, 
                        String.format("policy: %s", action.getPolicy().toString()),
                        7);
            }
            // connect source
            for (KPort source : actionSources.get(action)) {
                connect(createDelayEdge(action), source, ports.getKey());
            }
            
            // connect targets
            for (KPort target : actionDestinations.get(action)) {
                connect(createDelayEdge(action), ports.getValue(), target);
            }
        }
        
        // Transform connections.
        // First, collect all the source ports.
        List<PortInstance> sourcePorts = new LinkedList<>(reactorInstance.inputs);
        for (ReactorInstance child : reactorInstance.children) {
            sourcePorts.addAll(child.outputs);
        }

        for (PortInstance leftPort : sourcePorts) {
            KPort source = leftPort.getParent() == reactorInstance ? 
                    parentInputPorts.get(leftPort) : 
                    outputPorts.get(leftPort.getParent(), leftPort);
            
            for (SendRange sendRange : leftPort.getDependentPorts()) {
                for (RuntimeRange<PortInstance> rightRange : sendRange.destinations) {
                    PortInstance rightPort = rightRange.instance;
                    KPort target = rightPort.getParent() == reactorInstance ? 
                            parentOutputPorts.get(rightPort) : 
                            inputPorts.get(rightPort.getParent(), rightPort);
                    // There should be a connection, but skip if not.
                    Connection connection = sendRange.connection;
                    if (connection != null) {
                        KEdge edge = createIODependencyEdge(connection, (leftPort.isMultiport() || rightPort.isMultiport()));
                        if (connection.getDelay() != null) {
                            KLabel delayLabel = _kLabelExtensions.addCenterEdgeLabel(edge, ASTUtils.toText(connection.getDelay()));
                            associateWith(delayLabel, connection.getDelay());
                            if (connection.isPhysical()) {
                                _linguaFrancaStyleExtensions.applyOnEdgePysicalDelayStyle(delayLabel, 
                                        reactorInstance.isMainOrFederated() ? Colors.WHITE : Colors.GRAY_95);
                            } else {
                                _linguaFrancaStyleExtensions.applyOnEdgeDelayStyle(delayLabel);
                            }
                        } else if (connection.isPhysical()) {
                            KLabel physicalConnectionLabel = _kLabelExtensions.addCenterEdgeLabel(edge, "---");
                            _linguaFrancaStyleExtensions.applyOnEdgePysicalStyle(physicalConnectionLabel, 
                                    reactorInstance.isMainOrFederated() ? Colors.WHITE : Colors.GRAY_95);
                        }
                        if (source != null && target != null) {
                            // check for inside loop (direct in -> out connection with delay)
                            if (parentInputPorts.values().contains(source) &&
                                    parentOutputPorts.values().contains(target)) {
                                // edge.setLayoutOption(CoreOptions.INSIDE_SELF_LOOPS_YO, true) // Does not work as expected
                                // Introduce dummy node to enable direct connection (that is also hidden when collapsed)
                                KNode dummy = _kNodeExtensions.createNode();
                                if (directConnectionDummyNodes.containsKey(target)) {
                                    dummy = directConnectionDummyNodes.get(target);
                                } else {
                                    nodes.add(dummy);
                                    directConnectionDummyNodes.put(target, dummy);
                                    _kRenderingExtensions.addInvisibleContainerRendering(dummy);
                                    _kNodeExtensions.setNodeSize(dummy, 0, 0);
                                    KEdge extraEdge = createIODependencyEdge(null, 
                                            (leftPort.isMultiport() || rightPort.isMultiport()));
                                    connect(extraEdge, dummy, target);
                                }
                                connect(edge, source, dummy);
                            } else {
                                connect(edge, source, target);
                            }
                        }
                    }
                }
            }
        }

        // Add startup/shutdown
        if (startupUsed) {
            _linguaFrancaShapeExtensions.addStartupFigure(startupNode);
            nodes.add(0, startupNode);
            setLayoutOption(startupNode, LayeredOptions.LAYERING_LAYER_CONSTRAINT, LayerConstraint.FIRST);
            if (getBooleanValue(REACTIONS_USE_HYPEREDGES)) {
                KPort port = addInvisiblePort(startupNode);
                startupNode.getOutgoingEdges().forEach(it -> {
                    it.setSourcePort(port);
                });
            }
        }
        if (shutdownUsed) {
            _linguaFrancaShapeExtensions.addShutdownFigure(shutdownNode);
          nodes.add(0, shutdownNode);
          if (getBooleanValue(REACTIONS_USE_HYPEREDGES)) {  // connect all edges to one port
              KPort port = addInvisiblePort(shutdownNode);
              shutdownNode.getOutgoingEdges().forEach(it -> {
                  it.setSourcePort(port);
              });
            }
        }
        
        // Postprocess timer nodes
        if (getBooleanValue(REACTIONS_USE_HYPEREDGES)) { // connect all edges to one port
            for (KNode timerNode : timerNodes.values()) {
                KPort port = addInvisiblePort(timerNode);
                timerNode.getOutgoingEdges().forEach(it -> {
                    it.setSourcePort(port);
                });
            }
        }
        
        // Add reaction order edges (add last to have them on top of other edges)
        if (reactorInstance.reactions.size() > 1) {
            KNode prevNode = reactionNodes.get(IterableExtensions.head(reactorInstance.reactions));
            Iterable<KNode> iterList = IterableExtensions.map(
                    IterableExtensions.drop(reactorInstance.reactions, 1), 
                    reactionNodes::get);
            for (KNode node : iterList) {
                KEdge edge = createOrderEdge();
                edge.setSource(prevNode);
                edge.setTarget(node);
                edge.setProperty(CoreOptions.NO_LAYOUT, true);
                
                // Do not remove them, as they are needed for cycle detection
                KRendering edgeRendering = _kRenderingExtensions.getKRendering(edge);
                _kRenderingExtensions.setInvisible(edgeRendering, !getBooleanValue(SHOW_REACTION_ORDER_EDGES));
                _kRenderingExtensions.getInvisible(edgeRendering).setPropagateToChildren(true);
               // TODO this does not work work with incremental update (https://github.com/kieler/KLighD/issues/37)
               // if (!getBooleanValue(SHOW_REACTION_ORDER_EDGES)) edge.initiallyHide()
                
                prevNode = node;
            }
        }
        
        _modeDiagrams.handleModes(nodes, reactorInstance);
        
        return nodes;
    }
    
    private String createReactorLabel(ReactorInstance reactorInstance) {
        StringBuilder b = new StringBuilder();
        if (getBooleanValue(SHOW_INSTANCE_NAMES) && !_utilityExtensions.isRoot(reactorInstance)) {
            if (!reactorInstance.isMainOrFederated()) {
                b.append(reactorInstance.getName()).append(" : ");
            }
        }
        if (reactorInstance.isMainOrFederated()) {
            try {
                b.append(FileUtil.nameWithoutExtension(reactorInstance.reactorDeclaration.eResource()));
            } catch (Exception e) {
                throw Exceptions.sneakyThrow(e);
            }
        } else if (reactorInstance.reactorDeclaration == null) {
            // There is an error in the graph.
            b.append("<Unresolved Reactor>");
        } else {
            b.append(reactorInstance.reactorDeclaration.getName());
        }
        if (getObjectValue(REACTOR_PARAMETER_MODE) == ReactorParameterDisplayModes.TITLE) {
            if (reactorInstance.parameters.isEmpty()) {
                b.append("()");
            } else {
                b.append(IterableExtensions.join(reactorInstance.parameters, "(", ", ", ")", 
                        it -> {
                            return createParameterLabel(it, false);
                        }));
            }
        }
        return b.toString();
    }
    
    private void addParameterList(KContainerRendering container, List<ParameterInstance> parameters) {
        int cols = 1;
        try {
            cols = getIntValue(REACTOR_PARAMETER_TABLE_COLS);
        } catch (Exception e) {} // ignore
        if (cols > parameters.size()) {
            cols = parameters.size();
        }
        _kContainerRenderingExtensions.setGridPlacement(container, cols);
        for (ParameterInstance param : parameters) {
            KText paramText = _kContainerRenderingExtensions.addText(container, createParameterLabel(param, true));
            _kRenderingExtensions.setFontSize(paramText, 8);
            _kRenderingExtensions.setHorizontalAlignment(paramText, HorizontalAlignment.LEFT);
        }
    }
    
    private String createParameterLabel(ParameterInstance param, boolean bullet) {
        StringBuilder b = new StringBuilder();
        if (bullet) {
            b.append("\u2022 ");
        }
        b.append(param.getName());
        String t = param.type.toText();
        if (!StringExtensions.isNullOrEmpty(t)) {
            b.append(":").append(t);
        }
        if (!IterableExtensions.isNullOrEmpty(param.getInitialValue())) {
            b.append("(");
            b.append(IterableExtensions.join(param.getInitialValue(), ", ", _utilityExtensions::toText));
            b.append(")");
        }
        return b.toString();
    }
    
    private KEdge createDelayEdge(Object associate) {
        KEdge edge = _kEdgeExtensions.createEdge();
        associateWith(edge, associate);
        KPolyline line = _kEdgeExtensions.addPolyline(edge);
        _linguaFrancaStyleExtensions.boldLineSelectionStyle(line);
        _kPolylineExtensions.addJunctionPointDecorator(line);
        if (getBooleanValue(USE_ALTERNATIVE_DASH_PATTERN)) {
            _kRenderingExtensions.setLineStyle(line, LineStyle.CUSTOM);
            _kRenderingExtensions.getLineStyle(line).getDashPattern().addAll(ALTERNATIVE_DASH_PATTERN);
        } else {
            _kRenderingExtensions.setLineStyle(line, LineStyle.DASH);
        }
        return edge;
    }
    
    private KEdge createIODependencyEdge(Object associate, boolean multiport) {
        KEdge edge = _kEdgeExtensions.createEdge();
        if (associate != null) {
            associateWith(edge, associate);
        }
        KPolyline line = _kEdgeExtensions.addPolyline(edge);
        _linguaFrancaStyleExtensions.boldLineSelectionStyle(line);
        _kPolylineExtensions.addJunctionPointDecorator(line);
        if (multiport) {
            // Render multiport connections and bank connections in bold.
            _kRenderingExtensions.setLineWidth(line, 2.2f);
            _kRenderingExtensions.setLineCap(line, LineCap.CAP_SQUARE);
            // Adjust junction point size
            _kPolylineExtensions.setJunctionPointDecorator(line, line.getJunctionPointRendering(), 6, 6);
        }
        return edge;
    }
    
    private KEdge createDependencyEdge(Object associate) {
        KEdge edge = _kEdgeExtensions.createEdge();
        if (associate != null) {
            associateWith(edge, associate);
        }
        KPolyline line = _kEdgeExtensions.addPolyline(edge);
        _linguaFrancaStyleExtensions.boldLineSelectionStyle(line);
        _kPolylineExtensions.addJunctionPointDecorator(line);
        if (getBooleanValue(USE_ALTERNATIVE_DASH_PATTERN)) {
            _kRenderingExtensions.setLineStyle(line, LineStyle.CUSTOM);
            _kRenderingExtensions.getLineStyle(line).getDashPattern().addAll(ALTERNATIVE_DASH_PATTERN);
        } else {
            _kRenderingExtensions.setLineStyle(line, LineStyle.DASH);
        }
        return edge;
    }
    
    private KEdge createOrderEdge() {
        KEdge edge = _kEdgeExtensions.createEdge();
        KPolyline line = _kEdgeExtensions.addPolyline(edge);
        _kRenderingExtensions.setLineWidth(line, 1.5f);
        _kRenderingExtensions.setLineStyle(line, LineStyle.DOT);
        _kRenderingExtensions.setForeground(line, Colors.CHOCOLATE_1);
        _linguaFrancaStyleExtensions.boldLineSelectionStyle(line);
        //addFixedTailArrowDecorator() // Fix for bug: https://github.com/kieler/KLighD/issues/38
        _kPolylineExtensions.addHeadArrowDecorator(line);
        return edge;
    }
    
    private KEdge connect(KEdge edge, KNode src, KNode dst) {
        edge.setSource(src);
        edge.setTarget(dst);
        return edge;
    }
    
    private KEdge connect(KEdge edge, KNode src, KPort dst) {
        edge.setSource(src);
        edge.setTargetPort(dst);
        edge.setTarget(dst != null ? dst.getNode() : null);
        return edge;
    }
    
    private KEdge connect(KEdge edge, KPort src, KNode dst) {
        edge.setSourcePort(src);
        edge.setSource(src != null ? src.getNode() : null);
        edge.setTarget(dst);
        return edge;
    }
    
    private KEdge connect(KEdge edge, KPort src, KPort dst) {
        edge.setSourcePort(src);
        edge.setSource(src != null ? src.getNode() : null);
        edge.setTargetPort(dst);
        edge.setTarget(dst != null ? dst.getNode() : null);
        return edge;
    }
    
    /**
     * Translate an input/output into a port.
     */
    private KPort addIOPort(KNode node, PortInstance lfPort, boolean input, boolean multiport, boolean bank) {
        KPort port = _kPortExtensions.createPort();
        node.getPorts().add(port);
        associateWith(port, lfPort.getDefinition());
        NamedInstanceUtil.linkInstance(port, lfPort);
        _kPortExtensions.setPortSize(port, 6, 6);
        
        if (input) {
            // multiports are smaller by an offset at the right, hence compensate in inputs
            double offset = multiport ? -3.4 : -3.3;
            setLayoutOption(port, CoreOptions.PORT_SIDE, PortSide.WEST);
            setLayoutOption(port, CoreOptions.PORT_BORDER_OFFSET, offset);
        } else {
            double offset = multiport ? -2.6 : -3.3; // multiports are smaller
            offset = bank ? offset - LinguaFrancaShapeExtensions.BANK_FIGURE_X_OFFSET_SUM : offset; // compensate bank figure width
            setLayoutOption(port, CoreOptions.PORT_SIDE, PortSide.EAST);
            setLayoutOption(port, CoreOptions.PORT_BORDER_OFFSET, offset);
        }
        
        if (bank && !node.getProperty(REACTOR_HAS_BANK_PORT_OFFSET)) {// compensate bank figure height
            // https://github.com/eclipse/elk/issues/693
            _utilityExtensions.getPortMarginsInitIfAbsent(node).add(
                    new ElkMargin(0, 0, LinguaFrancaShapeExtensions.BANK_FIGURE_Y_OFFSET_SUM, 0));
            node.setProperty(REACTOR_HAS_BANK_PORT_OFFSET, true); // only once
        }
        
        _linguaFrancaShapeExtensions.addTrianglePort(port, multiport);
        
        String label = lfPort.getName();
        if (!getBooleanValue(SHOW_PORT_NAMES)) {
            label = "";
        }
        if (getBooleanValue(SHOW_MULTIPORT_WIDTH)) {
            if (lfPort.isMultiport()) {
                label += (lfPort.getWidth() >= 0) ? 
                        "[" + lfPort.getWidth() + "]" :
                        "[?]";
            }
        }
        associateWith(_kLabelExtensions.addOutsidePortLabel(port, label, 8), lfPort.getDefinition());
        return port;
    }

    private KPort addInvisiblePort(KNode node) {
        KPort port = _kPortExtensions.createPort();
        node.getPorts().add(port);
        port.setSize(0, 0); // invisible
        return port;
    }
    
    private KNode addErrorComment(KNode node, String message) {
        KNode comment = _kNodeExtensions.createNode();
        setLayoutOption(comment, CoreOptions.COMMENT_BOX, true);
        KRoundedRectangle commentFigure = _linguaFrancaShapeExtensions.addCommentFigure(comment, message);
        _linguaFrancaStyleExtensions.errorStyle(commentFigure);
        _kRenderingExtensions.setBackground(commentFigure, Colors.PEACH_PUFF_2);
        
        // connect
        KEdge edge = _kEdgeExtensions.createEdge();
        edge.setSource(comment);
        edge.setTarget(node);
        _linguaFrancaStyleExtensions.errorStyle(_linguaFrancaShapeExtensions.addCommentPolyline(edge));
        return comment;
    }
    
    private Iterable<KNode> createUserComments(EObject element, KNode targetNode) {
        if (getBooleanValue(SHOW_USER_LABELS)) {
            String commentText = ASTUtils.findAnnotationInComments(element, "@label");
            
            if (!StringExtensions.isNullOrEmpty(commentText)) {
                KNode comment = _kNodeExtensions.createNode();
                setLayoutOption(comment, CoreOptions.COMMENT_BOX, true);
                KRoundedRectangle commentFigure = _linguaFrancaShapeExtensions.addCommentFigure(comment, commentText);
                _linguaFrancaStyleExtensions.commentStyle(commentFigure);
                
                // connect
                KEdge edge = _kEdgeExtensions.createEdge();
                edge.setSource(comment);
                edge.setTarget(targetNode);
                _linguaFrancaStyleExtensions.commentStyle(_linguaFrancaShapeExtensions.addCommentPolyline(edge));
                
                return List.of(comment);
            }
        }
        return List.of();
    }

}
