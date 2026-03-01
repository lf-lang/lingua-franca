package org.lflang.diagram.synthesis.util;

import com.google.common.collect.HashMultimap;
import com.google.inject.Inject;
import de.cau.cs.kieler.klighd.kgraph.KEdge;
import de.cau.cs.kieler.klighd.kgraph.KGraphElement;
import de.cau.cs.kieler.klighd.kgraph.KNode;
import de.cau.cs.kieler.klighd.kgraph.KPort;
import de.cau.cs.kieler.klighd.krendering.ViewSynthesisShared;
import java.util.Map;
import java.util.Set;
import java.util.function.Consumer;
import org.eclipse.elk.graph.properties.Property;
import org.eclipse.xtext.xbase.lib.Extension;
import org.lflang.diagram.synthesis.AbstractSynthesisExtensions;
import org.lflang.generator.NamedInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Connection;

/**
 * Dependency cycle detection for Lingua Franca diagrams.
 *
 * @author Alexander Schulz-Rosengarten
 * @ingroup Diagram
 */
@ViewSynthesisShared
public class CycleVisualization extends AbstractSynthesisExtensions {

  // Properties for marking diagram elements
  public static final Property<Boolean> DEPENDENCY_CYCLE =
      new Property<>("org.lflang.diagram.synthesis.dependency.cycle", false);

  @Inject @Extension private UtilityExtensions _utilityExtensions;

  /**
   * Performs cycle detection based on the diagram's graph structure and applies given highlighting
   * to the included elements
   */
  public boolean detectAndHighlightCycles(
      ReactorInstance rootReactorInstance,
      Map<ReactorInstance, KNode> allReactorNodes,
      Consumer<KGraphElement> highlighter) {

    if (rootReactorInstance.hasCycles() && highlighter != null) {
      // Highlight cycles
      // A cycle consists of reactions and ports.
      HashMultimap<ReactorInstance, NamedInstance<?>> cycleElementsByReactor =
          HashMultimap.create();
      Set<NamedInstance<?>> cycles = rootReactorInstance.getCycles();
      for (NamedInstance<?> element : cycles) {
        // First find the involved reactor instances
        if (element instanceof ReactorInstance) {
          cycleElementsByReactor.put((ReactorInstance) element, element);
        } else {
          cycleElementsByReactor.put(element.getParent(), element);
        }
      }

      for (ReactorInstance reactor : cycleElementsByReactor.keySet()) {
        KNode node = allReactorNodes.get(reactor);
        if (node != null) {
          node.setProperty(DEPENDENCY_CYCLE, true);
          highlighter.accept(node);

          Set<NamedInstance<?>> reactorContentInCycle = cycleElementsByReactor.get(reactor);

          // Reactor edges
          for (KEdge edge : node.getOutgoingEdges()) {
            if (connectsCycleElements(edge, cycles)) {
              edge.setProperty(DEPENDENCY_CYCLE, true);
              highlighter.accept(edge);
            }
          }

          // Reactor ports
          for (KPort port : node.getPorts()) {
            if (reactorContentInCycle.contains(NamedInstanceUtil.getLinkedInstance(port))) {
              port.setProperty(DEPENDENCY_CYCLE, true);
              highlighter.accept(port);
            }
          }

          // Child Nodes
          for (KNode childNode : node.getChildren()) {
            if (reactorContentInCycle.contains(NamedInstanceUtil.getLinkedInstance(childNode))
                && !_utilityExtensions.sourceIsReactor(childNode)) {
              childNode.setProperty(DEPENDENCY_CYCLE, true);
              highlighter.accept(childNode);

              for (KEdge edge : childNode.getOutgoingEdges()) {
                if (connectsCycleElements(edge, cycles)) {
                  edge.setProperty(DEPENDENCY_CYCLE, true);
                  highlighter.accept(edge);
                }
              }
            }
          }
        }
      }
      return true;
    }
    return false;
  }

  /**
   * Checks whether an edge connects two elements that are part of the cycle. Assumes that the
   * source node is always part of the cycle!
   */
  private boolean connectsCycleElements(KEdge edge, Set<NamedInstance<?>> cycle) {
    return (
        // if source is not a reactor, there is nothing to check
        !_utilityExtensions.sourceIsReactor(edge.getSource())
            ||
            // otherwise, the source port must be on the cycle
            cycle.contains(NamedInstanceUtil.getLinkedInstance(edge.getSourcePort())))
        && (
        // leads to reactor port in cycle
        _utilityExtensions.sourceIsReactor(edge.getTarget())
                && cycle.contains(NamedInstanceUtil.getLinkedInstance(edge.getTargetPort()))
            ||
            // leads to reaction in cycle
            !_utilityExtensions.sourceIsReactor(edge.getTarget())
                && cycle.contains(NamedInstanceUtil.getLinkedInstance(edge.getTarget())))
        && (
        // Special case only for connections
        !(_utilityExtensions.sourceElement(edge) instanceof Connection)
            || (
            // If the edge represents a connections between two ports in the cycle (checked before),
            // then it is only included in the actual cycle, if it is neither delayed nor physical.
            ((Connection) _utilityExtensions.sourceElement(edge)).getDelay() == null
                && !((Connection) _utilityExtensions.sourceElement(edge)).isPhysical()));
  }
}
