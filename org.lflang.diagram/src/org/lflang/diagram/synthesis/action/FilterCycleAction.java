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
import java.util.List;
import java.util.WeakHashMap;
import java.util.function.Consumer;
import java.util.function.Predicate;
import org.eclipse.elk.graph.properties.Property;
import org.eclipse.emf.common.util.EList;
import org.eclipse.xtext.xbase.lib.Functions.Function1;
import org.eclipse.xtext.xbase.lib.IterableExtensions;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.eclipse.xtext.xbase.lib.ListExtensions;
import org.lflang.diagram.synthesis.LinguaFrancaSynthesis;
import org.lflang.diagram.synthesis.util.CycleVisualization;

/**
 * Action that filters the diagram for only those elements included in a cycle.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
public class FilterCycleAction extends AbstractAction {
    
    public static final String ID = "org.lflang.diagram.synthesis.action.FilterCycleAction";
    
    /**
     * Memory-leak-free cache of filtered states
     */
	private static final WeakHashMap<Object, Boolean> FILTERING_STATES = new WeakHashMap<Object, Boolean>();
	
    /**
	 * Property to mark filter button
	 */
	private static final Property<Boolean> FILTER_BUTTON = new Property<Boolean>("org.lflang.diagram.synthesis.action.cyclefilter.button", false);
    
	@Override
	public IAction.ActionResult execute(final IAction.ActionContext context) {
		ViewContext vc = context.getViewContext();
		Object all = vc.getOptionValue(LinguaFrancaSynthesis.SHOW_ALL_REACTORS);
		List<KNode> nodes = vc.getViewModel().getChildren();

		if (all instanceof Boolean && (Boolean) all) {
			nodes = IterableExtensions.<KNode>toList(
						Iterables.<KNode>concat(
							ListExtensions.<KNode, EList<KNode>>map(
								nodes, it -> { return it.getChildren(); }
							)
						)
					);
		}

		if (IterableExtensions.<KNode>exists(nodes, it -> { return this.isCycleFiltered(it); })) {
			// undo
			nodes.forEach(it -> this.resetCycleFiltering(it));
        	
			// re-synthesize everything
			vc.getViewModel().getChildren().clear();
      		vc.update();
		} else {
			// filter
			nodes.forEach(it -> {
				this.markCycleFiltered(it);
        		this.filterCycle(it);
			});

			Function1<KText, Boolean> filterButton =  it -> { 
				return it.<Boolean>getProperty(FilterCycleAction.FILTER_BUTTON); 
			};

			// switch filter label
			for (KNode node : IterableExtensions.<KNode>filter(nodes, filterButton)) {
				Iterator<KText> ktexts = Iterators.<KText>filter(node.eAllContents(), KText.class);
				KText text = IteratorExtensions.<KText>findFirst(ktexts, filterButton);
				if (text != null) {
					text.setText(LinguaFrancaSynthesis.TEXT_ERROR_CYCLE_BTN_UNFILTER);
				}
			}
		}
        
        return IAction.ActionResult.createResult(true);
    }
    
    public void filterCycle(KNode root) {
		Predicate<KNode> inCycle = it -> {
			return !it.<Boolean>getProperty(CycleVisualization.DEPENDENCY_CYCLE);
		};

		root.getChildren().removeIf(inCycle);
    	for (KNode node : root.getChildren()) {
    		node.getOutgoingEdges().removeIf(inCycle);
    		this.filterCycle(node);
    	}
    }

    public void markCycleFiltered(KNode node) {
    	Object source = this.sourceElement(node);
		if (source != null) {
			FilterCycleAction.FILTERING_STATES.put(source, Boolean.valueOf(true));
		}
    }
    
    public void resetCycleFiltering(KNode node) {
    	Object source = this.sourceElement(node);
		if (source != null) {
			FilterCycleAction.FILTERING_STATES.remove(source);
		}
    }  
      
    public boolean isCycleFiltered(KNode node) {
    	Object source = this.sourceElement(node);
		Boolean result = FilterCycleAction.FILTERING_STATES.get(source);
    	return result == null ? false : result;
    }
    
    public void markCycleFilterText(KText text, KNode node) {
    	text.<Boolean>setProperty(FilterCycleAction.FILTER_BUTTON, Boolean.valueOf(true));
    	node.<Boolean>setProperty(FilterCycleAction.FILTER_BUTTON, Boolean.valueOf(true));
    }
}
