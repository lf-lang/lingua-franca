package org.lflang.diagram.synthesis.action;

import de.cau.cs.kieler.klighd.IAction;
import de.cau.cs.kieler.klighd.ViewContext;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.util.ModelingUtil;
import java.util.Iterator;
import org.lflang.diagram.synthesis.util.NamedInstanceUtil;
import org.lflang.lf.Mode;

/**
 * Action that collapses (hides details) of all reactor nodes.
 *
 * @author Alexander Schulz-Rosengarten
 * @ingroup Diagram
 */
public class ExpandAllReactorsAction extends AbstractAction {

  public static final String ID = "org.lflang.diagram.synthesis.action.ExpandAllReactorsAction";

  @Override
  public IAction.ActionResult execute(final IAction.ActionContext context) {
    ViewContext vc = context.getViewContext();
    Iterator<KNode> nodes = ModelingUtil.eAllContentsOfType(vc.getViewModel(), KNode.class);

    while (nodes.hasNext()) {
      var node = nodes.next();
      if (sourceIs(node, Mode.class) || sourceIsReactor(node)) {
        MemorizingExpandCollapseAction.setExpansionState(
            node, NamedInstanceUtil.getLinkedInstance(node), vc.getViewer(), true);
      }
    }
    return IAction.ActionResult.createResult(true);
  }
}
