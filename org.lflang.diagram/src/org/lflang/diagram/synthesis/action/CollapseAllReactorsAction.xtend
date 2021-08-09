package org.lflang.diagram.synthesis.action

import de.cau.cs.kieler.klighd.IAction
import de.cau.cs.kieler.klighd.kgraph.KNode

import static extension de.cau.cs.kieler.klighd.util.ModelingUtil.*
import static extension org.lflang.diagram.synthesis.action.MemorizingExpandCollapseAction.*
import static extension org.lflang.diagram.synthesis.util.NamedInstanceUtil.*

/**
 * Action that expands (shows details) of all reactor nodes.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
class CollapseAllReactorsAction extends AbstractAction {
    
    public static val ID = "org.lflang.diagram.synthesis.action.CollapseAllReactorsAction"
    
    override execute(ActionContext context) {
        val vc = context.viewContext
        for (node : vc.viewModel.eAllContentsOfType(KNode).filter[sourceIsReactor].toIterable) {
        	if (!(node.sourceAsReactor().main || node.sourceAsReactor().federated)) { // Do not collapse main reactor
            	node.setExpansionState(node.linkedInstance, vc.viewer, false)
            }
        }
        return IAction.ActionResult.createResult(true);
    }
    
}
