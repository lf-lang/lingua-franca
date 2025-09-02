package org.lflang.diagram.synthesis.action;

import de.cau.cs.kieler.klighd.IAction;
import de.cau.cs.kieler.klighd.internal.util.KlighdInternalProperties;
import de.cau.cs.kieler.klighd.kgraph.KGraphElement;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import org.lflang.lf.Reactor;

/**
 * Abstract super class for diagram actions that provides some convince methods.
 *
 * @author Alexander Schulz-Rosengarten
 * @ingroup Diagram
 */
public abstract class AbstractAction implements IAction {
  public Object sourceElement(final KGraphElement elem) {
    return elem.getProperty(KlighdInternalProperties.MODEL_ELEMEMT);
  }

  public boolean sourceIs(KNode node, Class<?> clazz) {
    return clazz.isInstance(sourceElement(node));
  }

  public boolean sourceIsReactor(final KNode node) {
    return sourceElement(node) instanceof Reactor;
  }

  public Reactor sourceAsReactor(final KNode node) {
    return sourceIsReactor(node) ? (Reactor) sourceElement(node) : null;
  }
}
