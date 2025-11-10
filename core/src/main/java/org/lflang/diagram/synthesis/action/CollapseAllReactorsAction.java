package org.lflang.diagram.synthesis.action;

import de.cau.cs.kieler.klighd.IAction;
import de.cau.cs.kieler.klighd.ViewContext;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.util.ModelingUtil;
import java.util.Iterator;
import org.lflang.diagram.synthesis.util.NamedInstanceUtil;
import org.lflang.lf.Mode;

/**
 * Action that expands (shows details) of all reactor nodes.
 *
 * @author Alexander Schulz-Rosengarten
 * @ingroup Diagram
 */
public class CollapseAllReactorsAction extends AbstractAction {

  public static final String ID = "org.lflang.diagram.synthesis.action.CollapseAllReactorsAction";

  @Override
  public IAction.ActionResult execute(final IAction.ActionContext context) {
    ViewContext vc = context.getViewContext();
    Iterator<KNode> nodes = ModelingUtil.eAllContentsOfType(vc.getViewModel(), KNode.class);

    while (nodes.hasNext()) {
      var node = nodes.next();
      if (sourceIs(node, Mode.class)
          || (sourceIsReactor(node)
              && !(sourceAsReactor(node).isMain() || sourceAsReactor(node).isFederated()))) {
        MemorizingExpandCollapseAction.setExpansionState(
            node, NamedInstanceUtil.getLinkedInstance(node), vc.getViewer(), false);
      }
    }
    return IAction.ActionResult.createResult(true);
  }
}
