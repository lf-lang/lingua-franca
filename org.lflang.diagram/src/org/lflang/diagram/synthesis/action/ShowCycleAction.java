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

import com.google.common.collect.Iterables;
import de.cau.cs.kieler.klighd.IAction;
import de.cau.cs.kieler.klighd.ViewContext;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.util.ModelingUtil;
import java.util.HashSet;
import java.util.LinkedList;
import org.eclipse.xtext.xbase.lib.CollectionLiterals;
import org.eclipse.xtext.xbase.lib.Conversions;
import org.eclipse.xtext.xbase.lib.Functions.Function1;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.lflang.diagram.synthesis.util.CycleVisualization;
import org.lflang.diagram.synthesis.util.NamedInstanceUtil;
import org.lflang.generator.NamedInstance;

/**
 * Action that expands all reactor nodes that are included in a cycle.
 * 
 * @author{Alexander Schulz-Rosengarten <als@informatik.uni-kiel.de>}
 */
public class ShowCycleAction extends AbstractAction {
    
    public static final String ID = "org.lflang.diagram.synthesis.action.ShowCycleAction";

    private static final CollapseAllReactorsAction collapseAll = new CollapseAllReactorsAction();
    
    @Override
    public IAction.ActionResult execute(final IAction.ActionContext context) {
        ViewContext vc = context.getViewContext();
        
        // Collapse all
        ShowCycleAction.collapseAll.execute(context);
        
        // Expand only errors
        HashSet<KNode> cycleNodes = CollectionLiterals.<KNode>newHashSet();

        Iterator<KNode> knodes = ModelingUtil.<KNode>eAllContentsOfType(vc.getViewModel(), KNode.class);
        knodes = IteratorExtensions.<KNode>filter(knodes, it -> {
            return it.<Boolean>getProperty(CycleVisualization.DEPENDENCY_CYCLE) && this.sourceIsReactor(it);
        });
        Iterable<KNode> knodesIterable = IteratorExtensions.<KNode>toIterable(knodes);
        Iterables.<KNode>addAll(cycleNodes, knodesIterable);
        LinkedList<KNode> check = new LinkedList<>();
        check.addAll(cycleNodes);

        // include parents
        while (!check.isEmpty()) {
        	KNode parent = check.pop().getParent();
        	if (parent != null && !cycleNodes.contains(parent)) {
        		cycleNodes.add(parent);
                check.add(parent);
        	}
        }
        
        // expand
        for (KNode node : cycleNodes) {
            MemorizingExpandCollapseAction.setExpansionState(
                node, 
                NamedInstanceUtil.<NamedInstance<?>>getLinkedInstance(node), 
                vc.getViewer(), 
                true
            );
        }
        return IAction.ActionResult.createResult(true);
    }
    
}
