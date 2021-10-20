package org.lflang.diagram.synthesis.action

import de.cau.cs.kieler.klighd.IAction
import de.cau.cs.kieler.klighd.kgraph.KNode
import org.lflang.lf.Mode
import org.lflang.lf.Reactor

import static extension de.cau.cs.kieler.klighd.util.ModelingUtil.*
import static extension org.lflang.diagram.synthesis.action.MemorizingExpandCollapseAction.*
import static extension org.lflang.diagram.synthesis.util.NamedInstanceUtil.*

/**
 * Action that collapses (hides details) of all reactor nodes.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
class ExpandAllReactorsAction extends AbstractAction {
    
    public static val ID = "org.lflang.diagram.synthesis.action.ExpandAllReactorsAction"
    
    override execute(ActionContext context) {
        val vc = context.viewContext
        for (node : vc.viewModel.eAllContentsOfType(KNode).filter[sourceIs(Reactor) || sourceIs(Mode)].toIterable) {
            node.setExpansionState(node.linkedInstance, vc.viewer, true)
        }
        return IAction.ActionResult.createResult(true);
    }
    
}
