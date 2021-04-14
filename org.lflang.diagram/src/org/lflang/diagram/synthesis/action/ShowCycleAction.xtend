package org.lflang.diagram.synthesis.action

import de.cau.cs.kieler.klighd.IAction
import de.cau.cs.kieler.klighd.kgraph.KNode
import org.lflang.diagram.synthesis.LinguaFrancaSynthesis

import static extension de.cau.cs.kieler.klighd.util.ModelingUtil.*
import static extension org.lflang.diagram.synthesis.action.MemorizingExpandCollapseAction.*
import org.lflang.diagram.synthesis.util.CycleVisualization

/**
 * Action that expands all reactor nodes that are included in a cycle.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
class ShowCycleAction extends AbstractAction {
    
    public static val ID = "org.lflang.diagram.synthesis.action.ShowCycleAction"
    
    static val collapseAll = new CollapseAllReactorsAction()
    
    override execute(ActionContext context) {
        val vc = context.viewContext
        
        // Collapse all
        collapseAll.execute(context)
        
        // Expand only errors
        val cycleNodes = <KNode>newHashSet()
        cycleNodes += vc.viewModel.eAllContentsOfType(KNode).filter[
        	getProperty(CycleVisualization.DEPENDENCY_CYCLE) && sourceIsReactor
        ].toIterable
        // include parents
        val check = <KNode>newLinkedList(cycleNodes)
        while (!check.empty) {
        	val parent = check.pop.parent
        	if (parent !== null && !cycleNodes.contains(parent)) {
        		cycleNodes += parent
        		check += parent
        	}
        }
        // expand
        for (node : cycleNodes) {
            node.setExpansionState(node.getProperty(LinguaFrancaSynthesis.REACTOR_INSTANCE)?.crumb, vc.viewer, true)
        }
        
        return IAction.ActionResult.createResult(true);
    }
    
}
