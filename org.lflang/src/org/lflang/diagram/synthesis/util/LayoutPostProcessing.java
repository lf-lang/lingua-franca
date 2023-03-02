/*************
* Copyright (c) 2022, Kiel University.
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

import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

import org.eclipse.elk.alg.layered.components.ComponentOrderingStrategy;
import org.eclipse.elk.alg.layered.options.CrossingMinimizationStrategy;
import org.eclipse.elk.alg.layered.options.CycleBreakingStrategy;
import org.eclipse.elk.alg.layered.options.GreedySwitchType;
import org.eclipse.elk.alg.layered.options.LayeredOptions;
import org.eclipse.elk.alg.layered.options.OrderingStrategy;
import org.eclipse.elk.core.options.CoreOptions;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions;
import org.lflang.diagram.synthesis.LinguaFrancaSynthesis;
import org.lflang.generator.TriggerInstance.BuiltinTriggerVariable;

import de.cau.cs.kieler.klighd.SynthesisOption;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared;
import de.cau.cs.kieler.klighd.syntheses.DiagramSyntheses;

/**
 * Set layout configuration options for the Lingua Franca diagram synthesis.
 * 
 * @author Sören Domrös
 */
@ViewSynthesisShared
public class LayoutPostProcessing extends AbstractSynthesisExtensions {

    /** Synthesis option to control the order of nodes and edges by model order. */
    public static final String MODEL_ORDER_OPTION = "Model Order";
    /** Uses semi-automatic layout. */
    public static final String LEGACY = "Legacy";
    /** Only reactions are strictly ordered by their model order. */
    public static final String STRICT_REACTION_ONLY = "Reactions Only";
    /** Reactions and reactor are strictly ordered by their model order. */
    public static final String STRICT = "Reactions and Reactors";
    /** Reactions and reactors are ordered by their model order if no additional crossing are created. */
    public static final String TIE_BREAKER = "Optimize Crossings";
    /**
     * No crossing minimization is done at all. This requires that actions and timers are sorted based on their model
     * order.
     */
    public static final String FULL_CONTROL = "Full Control";
    
    
    public static final SynthesisOption MODEL_ORDER = 
            SynthesisOption.createChoiceOption(
                    MODEL_ORDER_OPTION,
                    Arrays.asList(TIE_BREAKER, STRICT_REACTION_ONLY, STRICT, FULL_CONTROL),
                    STRICT_REACTION_ONLY).setCategory(LinguaFrancaSynthesis.LAYOUT);

    /**
     * Comparator to sort KNodes based on the textual order of their linked instances.
     * 
     * Startup, reset and shutdown actions are not in the model and are handled separately:
     * Startup actions will always be first.
     * Reset actions follow after the startup action.
     * Shutdown is always sorted last. However, shutdown actions will not have a model order set and are, therefore,
     * implicitly ordered by their connection.
     */
    public static final Comparator<KNode> TEXTUAL_ORDER = new Comparator<KNode>() {

        @Override
        public int compare(KNode node1, KNode node2) {
            var pos1 = getTextPosition(node1);
            var pos2 = getTextPosition(node2);
            if (pos1 >= 0 && pos1 >= 0) {
                return Integer.compare(pos1, pos2); // textual order
            } else if (pos1 >= 0) {
                return -1; // unassociated elements last
            } else if (pos2 >= 0) {
                return 1; // unassociated elements last
            }
            return Integer.compare(node1.hashCode(), node2.hashCode()); // any stable order between unassociated elements
        }
        
        private int getTextPosition(KNode node) {
            var instance = NamedInstanceUtil.getLinkedInstance(node);
            if (instance != null) {
                var definition = instance.getDefinition();
                if (definition instanceof BuiltinTriggerVariable) {
                    // special handling for built-in triggers
                    switch(((BuiltinTriggerVariable)definition).type) {
                        case STARTUP: return 0; // first
                        case RESET: return 1; // second
                        case SHUTDOWN: return Integer.MAX_VALUE; // last
                    }
                } else if (definition instanceof EObject) {
                    var ast = NodeModelUtils.getNode((EObject) definition);
                    if (ast != null) {
                        return ast.getOffset();
                    }
                }
            }
            return -1;
        }
    };

    /**
     * Configures layout options for main reactor.
     * 
     * @param node The KNode of the main reactor.
     */
    public void configureMainReactor(KNode node) {
        configureReactor(node);
    }

    /**
     * Configures layout options for a reactor.
     * 
     * @param node The KNode of a reactor. 
     */
    public void configureReactor(KNode node) {
        String modelOrderStrategy = (String) getObjectValue(MODEL_ORDER);
        
        switch (modelOrderStrategy) {
            case LEGACY:
                // Otherwise nodes are not sorted if they are not connected
                DiagramSyntheses.setLayoutOption(node, CoreOptions.SEPARATE_CONNECTED_COMPONENTS, false);
                // Needed to enforce node positions.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CROSSING_MINIMIZATION_SEMI_INTERACTIVE, true);
                // Costs a little more time but layout is quick, therefore, we can do that.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.THOROUGHNESS, 100);
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CROSSING_MINIMIZATION_GREEDY_SWITCH_TYPE, GreedySwitchType.TWO_SIDED);
                break;
            case STRICT_REACTION_ONLY:
                // Only set model order for reactions.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CONSIDER_MODEL_ORDER_NO_MODEL_ORDER, true);
                // Do tie-breaking model order cycle breaking. 
                // Minimize number of backward edges but make decisions based on the model order if no greedy best alternative exists.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CYCLE_BREAKING_STRATEGY, CycleBreakingStrategy.GREEDY_MODEL_ORDER);
                // Before crossing minimization sort all nodes and edges/ports but also consider the node model order.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CONSIDER_MODEL_ORDER_STRATEGY, OrderingStrategy.NODES_AND_EDGES);
                // Separate connected components should be drawn separately.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.SEPARATE_CONNECTED_COMPONENTS, true);
                // Component order is enforced by looking at the minimum element with respect to model order of each component.
                // Remember that the startUp action is always the first node.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CONSIDER_MODEL_ORDER_COMPONENTS, ComponentOrderingStrategy.FORCE_MODEL_ORDER);
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.COMPACTION_CONNECTED_COMPONENTS, true);

                // Node order should not change during crossing minimization.
                // Since only reactions will have a model order set in this approach the order of reactions in their respective
                // separate connected components always respects the model order.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CROSSING_MINIMIZATION_FORCE_NODE_MODEL_ORDER, true);
                // Disable greedy switch since this does otherwise change the node order after crossing minimization.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CROSSING_MINIMIZATION_GREEDY_SWITCH_TYPE, GreedySwitchType.OFF);
                break;
            case STRICT:
                // Do tie-breaking model order cycle breaking. 
                // Minimize number of backward edges but make decisions based on the model order if no greedy best alternative exists.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CYCLE_BREAKING_STRATEGY, CycleBreakingStrategy.GREEDY_MODEL_ORDER);
                // Before crossing minimization sort all nodes and edges/ports but also consider the node model order.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CONSIDER_MODEL_ORDER_STRATEGY, OrderingStrategy.NODES_AND_EDGES);
                // Separate connected components should be drawn separately.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.SEPARATE_CONNECTED_COMPONENTS, true);
                // Component order is enforced by looking at the minimum element with respect to model order of each component.
                // Remember that the startUp action is always the first node.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CONSIDER_MODEL_ORDER_COMPONENTS, ComponentOrderingStrategy.FORCE_MODEL_ORDER);
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.COMPACTION_CONNECTED_COMPONENTS, true);
                
                // Node order should not change during crossing minimization.
                // Since only reactions and reactors will have a model order set in this approach the order of reactions and reactors in their respective
                // separate connected components always respects the model order.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CROSSING_MINIMIZATION_FORCE_NODE_MODEL_ORDER, true);
                // Disable greedy switch since this does otherwise change the node order after crossing minimization.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CROSSING_MINIMIZATION_GREEDY_SWITCH_TYPE, GreedySwitchType.OFF);
                break;
            case TIE_BREAKER:
                // Do tie-breaking model order cycle breaking. 
                // Minimize number of backward edges but make decisions based on the model order if no greedy best alternative exists.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CYCLE_BREAKING_STRATEGY, CycleBreakingStrategy.GREEDY_MODEL_ORDER);
                // Before crossing minimization sort all nodes and edges/ports but also consider the node model order.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CONSIDER_MODEL_ORDER_STRATEGY, OrderingStrategy.NODES_AND_EDGES);
                // Separate connected components should be drawn separately.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.SEPARATE_CONNECTED_COMPONENTS, true);
                // Component order is enforced by looking at the minimum element with respect to model order of each component.
                // Remember that the startUp action is always the first node.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CONSIDER_MODEL_ORDER_COMPONENTS, ComponentOrderingStrategy.FORCE_MODEL_ORDER);
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.COMPACTION_CONNECTED_COMPONENTS, true);
                // During crossing minimization 10 node order violations are regarded as important as 1 edge crossing.
                // In reality this chooses the best node order from all tries.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CONSIDER_MODEL_ORDER_CROSSING_COUNTER_NODE_INFLUENCE, 0.1);
                // Increase the number of tries with different starting configurations.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.THOROUGHNESS, 100);

                break;
            case FULL_CONTROL:
                // Do strict model order cycle breaking. This may introduce unnecessary backward edges.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CYCLE_BREAKING_STRATEGY, CycleBreakingStrategy.MODEL_ORDER);
                // Before crossing minimization sort all nodes and edges/ports but also consider the node model order.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CONSIDER_MODEL_ORDER_STRATEGY, OrderingStrategy.NODES_AND_EDGES);
                // Separate connected components should be drawn separately.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.SEPARATE_CONNECTED_COMPONENTS, true);
                // Component order is enforced by looking at the minimum element with respect to model order of each component.
                // Remember that the startUp action is always the first node.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CONSIDER_MODEL_ORDER_COMPONENTS, ComponentOrderingStrategy.FORCE_MODEL_ORDER);
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.COMPACTION_CONNECTED_COMPONENTS, true);
                // Disable all kinds of crossing minimization entirely. Just take what is in the model and just do it.
                // This requires that the list of nodes is not ordered by type, e.g. first all reactions, then all reactors, then all actions, ...
                // but by their model order. In other approaches ordering actions between the reactions has no effect.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CROSSING_MINIMIZATION_STRATEGY, CrossingMinimizationStrategy.NONE);
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CROSSING_MINIMIZATION_GREEDY_SWITCH_TYPE, GreedySwitchType.OFF);  
                
                break;
            default:
                // Do nothing.
        }
    }

    /**
     * Configures layout options for an action.
     * 
     * @param node The KNode of an action.
     */
    public void configureAction(KNode node) {
        String modelOrderStrategy = (String) getObjectValue(MODEL_ORDER);
        
        switch (modelOrderStrategy) {
            case STRICT_REACTION_ONLY:
            case STRICT:
            case TIE_BREAKER:
                // Actions have no model order since their ordering in the model cannot be compared to the order of
                // for example reactions since they are generally defined below in inputs/outputs and above the reactions.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CONSIDER_MODEL_ORDER_NO_MODEL_ORDER, true);
                break;
            case FULL_CONTROL:
                // Give actions a model order since they should be controllable too.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CONSIDER_MODEL_ORDER_NO_MODEL_ORDER, false);
                break;
            default:
                // Do nothing.
        }
    }

    /**
     * Configures layout options for a timer.
     * 
     * @param node The KNode of a timer.
     */
    public void configureTimer(KNode node) {
        String modelOrderStrategy = (String) getObjectValue(MODEL_ORDER);
        
        switch (modelOrderStrategy) {
            case STRICT_REACTION_ONLY:
            case STRICT:
            case TIE_BREAKER:
                // Timers have no model order since their ordering in the model cannot be compared to the order of
                // for example reactions since they are generally defined below in inputs/outputs and above the reactions.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CONSIDER_MODEL_ORDER_NO_MODEL_ORDER, true);
                break;
            case FULL_CONTROL:
                // Give timers a model order since they should be controllable too.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CONSIDER_MODEL_ORDER_NO_MODEL_ORDER, false);
                break;
            default:
                // Do nothing.
        }
    }

    /**
     * Configures layout options for a startup action.
     * 
     * @param node The KNode of a startup action.
     */
    public void configureStartUp(KNode node) {
        // Nothing should be done. Model order is considered per default value.
        // The actual ordering of this node has to be done in the synthesis.
    }

    /**
     * Configures layout options for a shutdown action.
     * 
     * @param node The KNode of a shutdown action.
     */
    public void configureShutDown(KNode node) {
        String modelOrderStrategy = (String) getObjectValue(MODEL_ORDER);
        
        switch (modelOrderStrategy) {
            case STRICT_REACTION_ONLY:
            case STRICT:
            case TIE_BREAKER:
            case FULL_CONTROL:
                // The shutdown node cannot have a high model order, since this would confuse cycle breaking.
                // It  also cannot have a low model order.
                // It should have none at all and the other nodes should define its position.
                // This is no problem since the shutdown node has only outgoing edges.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CONSIDER_MODEL_ORDER_NO_MODEL_ORDER, true);
                break;
            default:
                // Do nothing.
        }
    }

    /**
     * Configures layout options for a reaction.
     * Currently a reaction does not have internal behavior that is visualized and its order is always considered,
     * therefore, nothing needs to be done.
     * 
     * @param node The KNode of a reaction.
     */
    public void configureReaction(KNode node) {
        // Has no internal behavior and model order is set by default.
    }

    /**
     * Configures layout options for a dummy node.
     * 
     * @param node The KNode of a dummy node.
     */
    public void configureDummy(KNode node) {
        String modelOrderStrategy = (String) getObjectValue(MODEL_ORDER);
        
        switch (modelOrderStrategy) {
            case STRICT_REACTION_ONLY:
            case STRICT:
            case TIE_BREAKER:
            case FULL_CONTROL:
                // A dummy node has no model order.
                DiagramSyntheses.setLayoutOption(node, LayeredOptions.CONSIDER_MODEL_ORDER_NO_MODEL_ORDER, true);
                break;
            default:
                // Do nothing.
        }
    }

    /**
     * Orders a list of nodes by their corresponding linked instance if synthesis option for full control is enabled.
     * Ordering is done by the {@link #TEXTUAL_ORDER} comparator.
     * 
     * @param nodes List of KNodes to be ordered.
     */
    public void orderChildren(List<KNode> nodes) {
        String modelOrderStrategy = (String) getObjectValue(MODEL_ORDER);
        if (FULL_CONTROL.equals(modelOrderStrategy)) {
            nodes.sort(TEXTUAL_ORDER);
        }
    }
}
