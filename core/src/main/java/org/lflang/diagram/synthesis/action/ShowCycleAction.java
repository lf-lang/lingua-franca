package org.lflang.diagram.synthesis.action;

import de.cau.cs.kieler.klighd.IAction;
import de.cau.cs.kieler.klighd.ViewContext;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.util.ModelingUtil;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Set;
import org.eclipse.xtext.xbase.lib.IteratorExtensions;
import org.lflang.diagram.synthesis.util.CycleVisualization;
import org.lflang.diagram.synthesis.util.NamedInstanceUtil;

/**
 * Action that expands all reactor nodes that are included in a cycle.
 *
 * @author Alexander Schulz-Rosengarten
 * @ingroup Diagram
 */
public class ShowCycleAction extends AbstractAction {

  public static final String ID = "org.lflang.diagram.synthesis.action.ShowCycleAction";

  private static final CollapseAllReactorsAction collapseAll = new CollapseAllReactorsAction();

  @Override
  public IAction.ActionResult execute(final IAction.ActionContext context) {
    ViewContext vc = context.getViewContext();

    // Collapse all
    collapseAll.execute(context);

    // Expand only errors
    Iterator<KNode> knodes = ModelingUtil.eAllContentsOfType(vc.getViewModel(), KNode.class);

    // Filter out nodes that are not in cycle or not a reactor
    knodes =
        IteratorExtensions.filter(
            knodes,
            it -> {
              return it.getProperty(CycleVisualization.DEPENDENCY_CYCLE) && sourceIsReactor(it);
            });

    // Remove duplicates
    Set<KNode> cycleNodes = IteratorExtensions.toSet(knodes);

    // Include parents
    LinkedList<KNode> check = new LinkedList<>(cycleNodes);

    while (!check.isEmpty()) {
      KNode parent = check.pop().getParent();
      if (parent != null && !cycleNodes.contains(parent)) {
        cycleNodes.add(parent);
        check.add(parent);
      }
    }

    // Expand
    for (KNode node : cycleNodes) {
      MemorizingExpandCollapseAction.setExpansionState(
          node, NamedInstanceUtil.getLinkedInstance(node), vc.getViewer(), true);
    }
    return IAction.ActionResult.createResult(true);
  }
}
