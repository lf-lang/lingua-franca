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

import java.util.WeakHashMap;

import org.lflang.diagram.synthesis.util.InterfaceDependenciesVisualization;
import org.lflang.diagram.synthesis.util.NamedInstanceUtil;
import org.lflang.generator.NamedInstance;
import org.lflang.generator.ReactorInstance;

import com.google.common.base.Preconditions;

import de.cau.cs.kieler.klighd.IAction;
import de.cau.cs.kieler.klighd.IViewer;
import de.cau.cs.kieler.klighd.SynthesisOption;
import de.cau.cs.kieler.klighd.ViewContext;
import de.cau.cs.kieler.klighd.kgraph.KNode;

/**
 * Action for toggling collapse/expand state of reactors that memorizes the state and
 * allows correct initialization synthesis runs for the same model.
 * Prevents automatic collapsing of manually expanded nodes.
 * 
 * @author Alexander Schulz-Rosengarten
 */
public class MemorizingExpandCollapseAction extends AbstractAction {
    
    public static final String ID = "org.lflang.diagram.synthesis.action.MemorizingExpandCollapseAction";
    
    /**
     * The related synthesis option
     */
    public static final SynthesisOption MEMORIZE_EXPANSION_STATES = SynthesisOption.createCheckOption("Remember Collapsed/Expanded Reactors", true);
    
    /**
     * Memory-leak-free cache of expansion states
     */
    private static final WeakHashMap<Object, Boolean> EXPANSION_STATES = new WeakHashMap<>();
        
    /**
     * Sets the expansion state of a node and saves it for future synthesis.
     */
    public static void setExpansionState(final KNode node, final Object memorizableObj, final IViewer viewer, final boolean expand) {

        Preconditions.checkNotNull(node);
        
        // Store new state if activated
        if (((Boolean) viewer.getViewContext().getOptionValue(MEMORIZE_EXPANSION_STATES)) && memorizableObj != null) {
            if (memorizableObj instanceof NamedInstance<?>) {
                EXPANSION_STATES.put(((NamedInstance<?>) memorizableObj).uniqueID(), expand);
            } else {
                EXPANSION_STATES.put(memorizableObj, expand);
            }
        }
        
        // Apply state
        if (expand) {
            viewer.expand(node);
        } else {
            viewer.collapse(node);
        }
        
        // Handle edges that should only appear for one of the renderings
        InterfaceDependenciesVisualization.updateInterfaceDependencyVisibility(node, expand);
    }
    
    /**
     * @return the memorized expansion state of the given model element or null if not memorized
     */
    public static Boolean getExpansionState(final Object obj) {
        if (obj instanceof NamedInstance<?>) {
            return EXPANSION_STATES.get(((NamedInstance<?>) obj).uniqueID());
        }
        return EXPANSION_STATES.get(obj);
    }
    
    //-----------------------------------------------------------------------------------------------------------------
    
    @Override
    public IAction.ActionResult execute(final IAction.ActionContext context) {
        ViewContext vc = context.getViewContext();
        IViewer v = vc.getViewer();
        KNode node = context.getKNode();
        NamedInstance<?> linkedInstance = NamedInstanceUtil.getLinkedInstance(node);
        
        // Find node that is properly linked
        while(node != null && linkedInstance == null) {
            node = node.getParent();
            linkedInstance = NamedInstanceUtil.getLinkedInstance(node);
        }
        
        if (node == null || (linkedInstance instanceof ReactorInstance && ((ReactorInstance) linkedInstance).isMainOrFederated())) {
            return IAction.ActionResult.createResult(false);
        } else {
            setExpansionState(node, linkedInstance, v, !v.isExpanded(node)); // toggle
            return IAction.ActionResult.createResult(true);
        }
    }
    
}
