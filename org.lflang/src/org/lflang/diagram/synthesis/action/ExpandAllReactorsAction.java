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

import de.cau.cs.kieler.klighd.IAction;
import de.cau.cs.kieler.klighd.ViewContext;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.util.ModelingUtil;
import java.util.Iterator;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.lflang.diagram.synthesis.util.NamedInstanceUtil;
import org.lflang.lf.Mode;

/**
 * Action that collapses (hides details) of all reactor nodes.
 * 
 * @author Alexander Schulz-Rosengarten
 */
public class ExpandAllReactorsAction extends AbstractAction {

    public static final String ID = "org.lflang.diagram.synthesis.action.ExpandAllReactorsAction";
    
    @Override
    public IAction.ActionResult execute(final IAction.ActionContext context) {
        ViewContext vc = context.getViewContext();
        Iterator<KNode> nodes = ModelingUtil.eAllContentsOfType(vc.getViewModel(), KNode.class);
        
        while(nodes.hasNext()) {
            var node = nodes.next();
            if (sourceIs(node, Mode.class) || sourceIsReactor(node)) {
                MemorizingExpandCollapseAction.setExpansionState(
                        node, 
                        NamedInstanceUtil.getLinkedInstance(node), 
                        vc.getViewer(), 
                        true
                );
            }
        }
        return IAction.ActionResult.createResult(true);
    }
}
  