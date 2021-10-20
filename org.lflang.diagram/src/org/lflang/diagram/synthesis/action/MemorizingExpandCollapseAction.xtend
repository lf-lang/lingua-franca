/*
 * KIELER - Kiel Integrated Environment for Layout Eclipse RichClient
 *
 * http://rtsys.informatik.uni-kiel.de/kieler
 * 
 * Copyright 2017 by
 * + Kiel University
 *   + Department of Computer Science
 *     + Real-Time and Embedded Systems Group
 * 
 * This code is provided under the terms of the Eclipse Public License (EPL).
 */
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
    
    public static val MEM_EXPAND_COLLAPSE_ACTION_ID = "org.lflang.diagram.synthesis.action.MemorizingExpandCollapseAction"
    
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
        var node = context.KNode
        
        while(node !== null && node.linkedInstance === null) {
            node = node.parent
        }
        if (node === null) {
            return IAction.ActionResult.createResult(false);
        } else {
            node.setExpansionState(node.linkedInstance, v, !v.isExpanded(node)) // toggle
            return IAction.ActionResult.createResult(true);
        }
        
    }
    
}
