package org.icyphy.linguafranca.diagram.synthesis.action

import de.cau.cs.kieler.klighd.IAction
import de.cau.cs.kieler.klighd.kgraph.KNode
import org.icyphy.linguafranca.diagram.synthesis.LinguaFrancaSynthesis
import org.icyphy.linguafranca.diagram.synthesis.LinguaFrancaSynthesisCycleDetection

import static extension de.cau.cs.kieler.klighd.util.ModelingUtil.*
import static extension org.icyphy.linguafranca.diagram.synthesis.action.MemorizingExpandCollapseAction.*

/**
 * Action that expands all reactor nodes that are included in a cycle.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
class ShowCycleAction extends AbstractAction {
    
    public static val ID = "org.icyphy.linguafranca.diagram.synthesis.action.ShowCycleAction"
    
    static val collapseAll = new CollapseAllReactorsAction()
    
    override execute(ActionContext context) {
        val vc = context.viewContext
        
        // Collapse all
        collapseAll.execute(context)
        
        // Expand only errors
        val cycleNodes = <KNode>newHashSet()
        cycleNodes += vc.viewModel.eAllContentsOfType(KNode).filter[
        	getProperty(LinguaFrancaSynthesisCycleDetection.DEPENDENCY_CYCLE) && sourceIsReactor
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
