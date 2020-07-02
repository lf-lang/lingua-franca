package org.icyphy.linguafranca.diagram.synthesis.action

import de.cau.cs.kieler.klighd.IAction
import de.cau.cs.kieler.klighd.kgraph.KNode
import org.icyphy.linguafranca.diagram.synthesis.LinguaFrancaSynthesis

import static extension de.cau.cs.kieler.klighd.util.ModelingUtil.*
import static extension org.icyphy.linguafranca.diagram.synthesis.action.MemorizingExpandCollapseAction.*

/**
 * Action that expands (shows details) of all reactor nodes.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
class CollapseAllReactorsAction extends AbstractAction {
    
    public static val ID = "org.icyphy.linguafranca.diagram.synthesis.action.CollapseAllReactorsAction"
    
    override execute(ActionContext context) {
        val vc = context.viewContext
        for (node : vc.viewModel.eAllContentsOfType(KNode).filter[sourceIsReactor].toIterable) {
        	if (!node.sourceAsReactor().main) { // Do not collapse main reactor
            	node.setExpansionState(node.getProperty(LinguaFrancaSynthesis.REACTOR_INSTANCE)?.crumb, vc.viewer, false)
            }
        }
        return IAction.ActionResult.createResult(true);
    }
    
}
