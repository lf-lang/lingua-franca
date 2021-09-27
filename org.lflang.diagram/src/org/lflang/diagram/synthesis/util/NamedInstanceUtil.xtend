package org.lflang.diagram.synthesis.util

import de.cau.cs.kieler.klighd.kgraph.KGraphElement
import org.eclipse.elk.graph.properties.Property
import org.lflang.generator.NamedInstance

/**
 * Utility class to link KGraphElements to NamedInstances.
 * 
 * @author als
 */
class NamedInstanceUtil {

    public static val LINKED_INSTANCE = new Property<NamedInstance<?>>(
        "org.lflang.linguafranca.diagram.synthesis.graph.instance")

    /**
     * Establishes a link between KGraphElement and NamedInstance.
     */
    static def linkInstance(KGraphElement elem, NamedInstance<?> instance) {
        elem.setProperty(LINKED_INSTANCE, instance)
    }

    /**
     * Returns the linked NamedInstance for ther given KGraphElement.
     */
    static def <T extends NamedInstance<?>> T getLinkedInstance(KGraphElement elem) {
        val instance = elem.getProperty(LINKED_INSTANCE)
        if (instance !== null) {
            return instance as T
        }
        return null
    }
}
