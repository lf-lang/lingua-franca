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
package org.lflang.diagram.synthesis.action;

import com.google.common.collect.Iterables;
import com.google.common.collect.Iterators;
import de.cau.cs.kieler.klighd.IAction;
import de.cau.cs.kieler.klighd.ViewContext;
import de.cau.cs.kieler.klighd.kgraph.KEdge;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.krendering.KText;

import java.util.Iterator;
import java.util.List;
import java.util.WeakHashMap;
import java.util.function.Predicate;
import org.eclipse.elk.graph.properties.Property;
import org.eclipse.xtext.xbase.lib.Functions.Function1;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.eclipse.xtext.xbase.lib.ListExtensions;
import org.lflang.diagram.synthesis.LinguaFrancaSynthesis;
import org.lflang.diagram.synthesis.util.CycleVisualization;

/**
 * Action that filters the diagram for only those elements included in a cycle.
 * 
 * @author Alexander Schulz-Rosengarten
 */
public class FilterCycleAction extends AbstractAction {
    
    public static final String ID = "org.lflang.diagram.synthesis.action.FilterCycleAction";
    
    /**
     * Memory-leak-free cache of filtered states
     */
    private static final WeakHashMap<Object, Boolean> FILTERING_STATES = new WeakHashMap<>();
    
    /**
     * INTERNAL property to mark filter button.
     */
    public static final Property<Boolean> FILTER_BUTTON = new Property<>("org.lflang.diagram.synthesis.action.cyclefilter.button", false);
    
    @Override
    public IAction.ActionResult execute(final IAction.ActionContext context) {
        ViewContext vc = context.getViewContext();
        Object all = vc.getOptionValue(LinguaFrancaSynthesis.SHOW_ALL_REACTORS);
        List<KNode> nodes = vc.getViewModel().getChildren();

        if (all instanceof Boolean && (Boolean) all) {
            nodes = IterableExtensions.toList(
                        Iterables.concat(
                            ListExtensions.map(
                                nodes, it -> { return it.getChildren(); }
                            )
                        )
                    );
        }

        if (IterableExtensions.exists(nodes, this::isCycleFiltered)) {
            // undo
            nodes.forEach(this::resetCycleFiltering);
            
            // re-synthesize everything
            vc.getViewModel().getChildren().clear();
              vc.update();
        } else {
            // filter
            nodes.forEach(it -> {
                this.markCycleFiltered(it);
                this.filterCycle(it);
            });

            Function1<KNode, Boolean> knodeFilterButton = it -> { 
                return it.getProperty(FILTER_BUTTON); 
            };
            
            Function1<KText, Boolean> ktextFilterButton = it -> { 
                return it.getProperty(FILTER_BUTTON); 
            };

            // switch filter label
            for (KNode node : IterableExtensions.filter(nodes, knodeFilterButton)) {
                Iterator<KText> ktexts = Iterators.filter(node.eAllContents(), KText.class);
                KText text = IteratorExtensions.findFirst(ktexts, ktextFilterButton);
                if (text != null) {
                    text.setText(LinguaFrancaSynthesis.TEXT_ERROR_CYCLE_BTN_UNFILTER);
                }
            }
        }
        
        return IAction.ActionResult.createResult(true);
    }
    
    public void filterCycle(KNode root) {
        Predicate<KNode> knodeNotInCycle = it -> {
            return !it.getProperty(CycleVisualization.DEPENDENCY_CYCLE);
        };
        
        Predicate<KEdge> kedgeNotInCycle = it -> {
            return !it.getProperty(CycleVisualization.DEPENDENCY_CYCLE);
        };
        
        root.getChildren().removeIf(knodeNotInCycle);
        for (KNode node : root.getChildren()) {
            node.getOutgoingEdges().removeIf(kedgeNotInCycle);
            this.filterCycle(node);
        }
    }

    public void markCycleFiltered(KNode node) {
        Object source = this.sourceElement(node);
        if (source != null) {
            FILTERING_STATES.put(source, true);
        }
    }
    
    public void resetCycleFiltering(KNode node) {
        Object source = this.sourceElement(node);
        if (source != null) {
            FILTERING_STATES.remove(source);
        }
    }

    public boolean isCycleFiltered(KNode node) {
        Object source = this.sourceElement(node);
        Boolean result = FILTERING_STATES.get(source);
        return result == null ? false : result;
    }

    public void markCycleFilterText(KText text, KNode node) {
        text.setProperty(FILTER_BUTTON, true);
        node.setProperty(FILTER_BUTTON, true);
    }
}
