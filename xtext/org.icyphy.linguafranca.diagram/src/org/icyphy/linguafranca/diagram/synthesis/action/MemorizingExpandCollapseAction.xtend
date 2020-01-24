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
package org.icyphy.linguafranca.diagram.synthesis.action

import de.cau.cs.kieler.klighd.IAction
import de.cau.cs.kieler.klighd.IViewer
import de.cau.cs.kieler.klighd.SynthesisOption
import de.cau.cs.kieler.klighd.kgraph.KNode
import java.util.WeakHashMap
import org.eclipse.emf.ecore.EObject

import static extension com.google.common.base.Preconditions.*
import org.icyphy.linguaFranca.Reactor
import org.icyphy.linguafranca.diagram.synthesis.LinguaFrancaSynthesis

class MemorizingExpandCollapseAction implements IAction {
    
    public static val MEM_EXPAND_COLLAPSE_ACTION_ID = "org.icyphy.linguafranca.diagram.synthesis.action.MemorizingExpandCollapseAction"
    
    /** The related synthesis option */
    public static val SynthesisOption MEMORIZE_EXPANSION_STATES = SynthesisOption.createCheckOption("Remember Collapsed/Expanded Reactors", true)
    
    /** Memory-leak-free cache of expansion states */
    public static final WeakHashMap<EObject, Boolean> EXPANSION_STATES = new WeakHashMap()
    
    /**
     * Sets the expansion state of a node and saves it for future synthesis.
     */
    static def setExpansionState(KNode node, EObject modelElement, IViewer viewer, boolean expand) {
        node.checkNotNull
        
        // Store new state if activated
        if (viewer.viewContext.getOptionValue(MEMORIZE_EXPANSION_STATES) as Boolean && modelElement !== null) {
            EXPANSION_STATES.put(modelElement, expand)
        }
        
        // Apply state
        if (expand) {
            viewer.expand(node)
        } else {
            viewer.collapse(node)
        }
    }
    
    /**
     * @return the memorized expansion state of the given model element or null if not memorized
     */
    static def getExpansionState(EObject modelElement) {
        return EXPANSION_STATES.get(modelElement)
    }
    
    //-----------------------------------------------------------------------------------------------------------------
    
    override execute(ActionContext context) {
        val vc = context.viewContext
        val v = vc.viewer 
        val node = context.KNode
        val source = vc.getSourceElement(node)
        
        if (source instanceof EObject) {
        	node.setExpansionState(node.getProperty(LinguaFrancaSynthesis.REACTOR_INSTANCE)?:source, v, !v.isExpanded(node)) // toggle
        }
        
        return IAction.ActionResult.createResult(true);
    }
    
}
