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
import de.cau.cs.kieler.klighd.IViewer
import de.cau.cs.kieler.klighd.SynthesisOption
import de.cau.cs.kieler.klighd.kgraph.KNode
import java.util.WeakHashMap
import org.lflang.generator.NamedInstance

import static extension com.google.common.base.Preconditions.*
import static extension org.lflang.diagram.synthesis.util.InterfaceDependenciesVisualization.updateInterfaceDependencyVisibility
import static extension org.lflang.diagram.synthesis.util.NamedInstanceUtil.*

/**
 * Action for toggling collapse/expand state of reactors that memorizes the state and
 * allows correct initialization synthesis runs for the same model.
 * Prevents automatic collapsing of manually expanded nodes.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
class MemorizingExpandCollapseAction extends AbstractAction {
    
    public static val ID = "org.lflang.diagram.synthesis.action.MemorizingExpandCollapseAction"
    
    /** The related synthesis option */
    public static val SynthesisOption MEMORIZE_EXPANSION_STATES = SynthesisOption.createCheckOption("Remember Collapsed/Expanded Reactors", true)
    
    /** Memory-leak-free cache of expansion states */
    static final WeakHashMap<Object, Boolean> EXPANSION_STATES = new WeakHashMap()
        
    /**
     * Sets the expansion state of a node and saves it for future synthesis.
     */
    static def setExpansionState(KNode node, Object memorizableObj, IViewer viewer, boolean expand) {
        node.checkNotNull
        
        // Store new state if activated
        if (viewer.viewContext.getOptionValue(MEMORIZE_EXPANSION_STATES) as Boolean && memorizableObj !== null) {
            if (memorizableObj instanceof NamedInstance) {
                EXPANSION_STATES.put(memorizableObj.uniqueID, expand)
            } else {
                EXPANSION_STATES.put(memorizableObj, expand)
            }
        }
        
        // Apply state
        if (expand) {
            viewer.expand(node)
        } else {
            viewer.collapse(node)
        }
        
        // Handle edges that should only appear for one of the renderings
        node.updateInterfaceDependencyVisibility(expand)
    }
    
    /**
     * @return the memorized expansion state of the given model element or null if not memorized
     */
    static def getExpansionState(Object obj) {
        if (obj instanceof NamedInstance) {
            return EXPANSION_STATES.get(obj.uniqueID)
        }
        return EXPANSION_STATES.get(obj)
    }
    
    //-----------------------------------------------------------------------------------------------------------------
    
    override execute(ActionContext context) {
        val vc = context.viewContext
        val v = vc.viewer 
        val node = context.KNode
        
        node.setExpansionState(node.linkedInstance, v, !v.isExpanded(node)) // toggle
        
        return IAction.ActionResult.createResult(true);
    }
    
}
