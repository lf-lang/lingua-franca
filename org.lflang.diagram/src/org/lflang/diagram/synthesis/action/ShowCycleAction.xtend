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
package org.lflang.diagram.synthesis.action

import de.cau.cs.kieler.klighd.IAction
import de.cau.cs.kieler.klighd.kgraph.KNode
import org.lflang.diagram.synthesis.util.CycleVisualization

import static extension de.cau.cs.kieler.klighd.util.ModelingUtil.*
import static extension org.lflang.diagram.synthesis.action.MemorizingExpandCollapseAction.*
import static extension org.lflang.diagram.synthesis.util.NamedInstanceUtil.*

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
            node.setExpansionState(node.linkedInstance, vc.viewer, true)
        }
        
        return IAction.ActionResult.createResult(true);
    }
    
}
