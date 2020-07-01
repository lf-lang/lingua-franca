package org.icyphy.linguafranca.diagram.synthesis.action

import de.cau.cs.kieler.klighd.IAction
import de.cau.cs.kieler.klighd.kgraph.KNode
import de.cau.cs.kieler.klighd.krendering.KText
import java.util.WeakHashMap
import org.eclipse.elk.graph.properties.Property
import org.icyphy.linguafranca.diagram.synthesis.LinguaFrancaSynthesis
import org.icyphy.linguafranca.diagram.synthesis.LinguaFrancaSynthesisCycleDetection

/**
 * Action that filters the diagram for only those elements included in a cycle.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
class FilterCycleAction extends AbstractAction {
    
    public static val ID = "org.icyphy.linguafranca.diagram.synthesis.action.FilterCycleAction"
    
    /** Memory-leak-free cache of filtered states */
    static final WeakHashMap<Object, Boolean> FILTERING_STATES = new WeakHashMap()
    /** Property to mark filter button */
	static val FILTER_BUTTON = new Property<Boolean>("org.icyphy.linguafranca.diagram.synthesis.action.cyclefilter.button", false)
    
    override execute(ActionContext context) {
        val vc = context.viewContext
        
        val all = vc.getOptionValue(LinguaFrancaSynthesis.SHOW_ALL_REACTORS)
        val nodes = if (all instanceof Boolean && all as Boolean) {
        	vc.viewModel.children.head.children
        } else {
        	vc.viewModel.children
        }
        
        if (nodes.exists[isCycleFiltered()]) {
        	// undo
        	nodes.forEach[resetCycleFiltering()]
        	
        	// re-synthesize everything
        	vc.viewModel.children.clear()
        	vc.update()
        } else {
        	// filter
        	nodes.forEach[markCycleFiltered(); filterCycle()]
        	
        	// switch filter label
        	for (node : nodes.filter[getProperty(FILTER_BUTTON)]) {
	        	val text = node.eAllContents.filter(KText).findFirst[getProperty(FILTER_BUTTON)]
	        	if (text !== null) {
	        		text.text = LinguaFrancaSynthesis.TEXT_ERROR_CYCLE_BTN_UNFILTER
	        	}
	        }
        }
        
        return IAction.ActionResult.createResult(true);
    }
    
    def void filterCycle(KNode root) {
    	root.children.removeIf[!getProperty(LinguaFrancaSynthesisCycleDetection.DEPENDENCY_CYCLE)]
    	for (node : root.children) {
    		node.outgoingEdges.removeIf[!getProperty(LinguaFrancaSynthesisCycleDetection.DEPENDENCY_CYCLE)]
    		node.filterCycle()
    	}
    }

    def void markCycleFiltered(KNode node) {
    	val source = node.sourceElement
    	if (source !== null) {
	    	FILTERING_STATES.put(source, true)
    	}
    }
    
    def void resetCycleFiltering(KNode node) {
    	val source = node.sourceElement
    	if (source !== null) {
	    	FILTERING_STATES.remove(source)
    	}
    }  
      
    def boolean isCycleFiltered(KNode node) {
    	val source = node.sourceElement
    	return FILTERING_STATES.get(source)?:false
    }
    
    def void markCycleFilterText(KText text, KNode node) {
    	text.setProperty(FILTER_BUTTON, true)
    	node.setProperty(FILTER_BUTTON, true)
    }
}
