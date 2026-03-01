package org.lflang.diagram.synthesis.util;

import de.cau.cs.kieler.klighd.kgraph.KGraphElement;
import org.eclipse.elk.graph.properties.IPropertyHolder;
import org.eclipse.elk.graph.properties.Property;
import org.lflang.generator.NamedInstance;

/**
 * Utility class to link KGraphElements to NamedInstances.
 *
 * @author Alexander Schulz-Rosengarten
 * @ingroup Diagram
 */
public class NamedInstanceUtil {
  public static final Property<NamedInstance<?>> LINKED_INSTANCE =
      new Property<>("org.lflang.linguafranca.diagram.synthesis.graph.instance");

  /** Establishes a link between KGraphElement and NamedInstance. */
  public static IPropertyHolder linkInstance(KGraphElement elem, NamedInstance<?> instance) {
    return elem.setProperty(LINKED_INSTANCE, instance);
  }

  /** Returns the linked NamedInstance for the given KGraphElement. */
  public static NamedInstance<?> getLinkedInstance(KGraphElement elem) {
    var instance = elem.getProperty(LINKED_INSTANCE);
    if (instance != null) {
      return instance;
    }
    return null;
  }
}
