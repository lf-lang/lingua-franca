package org.lflang.generator.c;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.Stack;
import org.lflang.TimeValue;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.RuntimeRange;
import org.lflang.generator.SendRange;
import org.lflang.lf.impl.CodeExprImpl;

/**
 * A Graph class to represent the connection topology of enclaves. The Nodes of this graph are
 * ReactorInstances and the Edges are Connection objects which contains information about the nature
 * of the connection and its associated delay. The graph is used for two purposes: 1) Verify the
 * absence of zero-delay enclave cycles. They are theoretically possible to support through PTAGs
 * (as in Federations) but are for now disallowed. 2) Use connection info to code-generate the
 * connection arrays used by the runtime to coordinate the timelines of the enclaves.
 */
public class CEnclaveGraph {
  private final Map<ReactorInstance, Set<EnclaveConnection>> graph;

  /** A stack containing the zeroDelayCycle, if it is found */
  private Stack<ReactorInstance> zeroDelayCycle;

  /**
   * A record representing an edge in the graph. It contains info on the connection between two
   * enclaves.
   */
  public record EnclaveConnection(
      ReactorInstance source,
      ReactorInstance target,
      TimeValue delay,
      boolean hasAfterDelay,
      boolean isPhysical) {}
  /**
   * The constructor. Given a List of enclaves, after the AST transformation, construct the
   * connection graph. It is important that this is constructed AFTER the
   * CEnclavedReactorTransformation AST is performed as we are finding the ConnectionReactors to
   * deduce the type of connection. This works even if some enclaves are not connected through their
   * top-level input or output ports. E.g. a reactor r1 contained within enclave e1 can be connected
   * to another enclave e2 also contained within e1. In this case we will find the connection
   * between e1 and e2 by iterating through the input ports of e2 which will lead os to an output
   * port of r1 which is within e1.
   *
   * @param enclaves The list of enclave instances in the program (included the main reactor)
   */
  public CEnclaveGraph(List<ReactorInstance> enclaves) {
    graph = new HashMap<>();
    addNodes(enclaves);

    // Iterate through all the enclaves and add nodes and edges to the graph.
    for (ReactorInstance enclave : enclaves) {
      // First search all inputs to the enclave and find if they are connected to other enclaves.
      for (PortInstance input : enclave.inputs) {
        if (input.eventualSources().size() == 1) {
          ReactorInstance source = input.eventualSources().get(0).parentReactor();
          ReactorInstance sourceEnclave = source.enclaveTop;

          // Get the delay by inspecting the ConnectionReactor
          if (input.getDependentPorts().size() == 1) {
            ReactorInstance connReactor =
                input.getDependentPorts().get(0).destinations.get(0).parentReactor();
            TimeValue delay = connReactor.actions.get(0).getMinDelay();
            boolean hasAfterDelay =
                ((CodeExprImpl)
                        connReactor
                            .getParameter("has_after_delay")
                            .getActualValue()
                            .getExprs()
                            .get(0))
                    .getCode()
                    .getBody()
                    .equals("true");
            addEdge(sourceEnclave, enclave, delay, hasAfterDelay, false);
          } else if (input.getDependentPorts().size() > 1) {
            throw new RuntimeException(
                "Enclave Input Port has more than 1 dependent ports. Should only be a single"
                    + " ConnectionReactor within this enclave");
          }
        } else if (input.eventualSources().size() > 1) {
          throw new RuntimeException(
              "Enclave had input port with more than one eventual source which is disallowed for"
                  + " enclaves.");
        }
      }

      // For each output port of the enclave. Follow the connection to the destination and add
      // an edge to that enclave. An Output port is always connected to a ConnectionReactor inside
      // the destination enclave.
      for (PortInstance output : enclave.outputs) {
        // Note that we iterate over the eventual destinations which should always take us to
        // ConnectionReactor within the destination enclave.
        for (SendRange sendRange : output.eventualDestinations()) {
          for (RuntimeRange runtimeRange : sendRange.destinations) {
            ReactorInstance destReactor = runtimeRange.parentReactor();
            ReactorInstance destEnclave = destReactor.enclaveTop;
            // destReactor is here actually the ConnectionReactor within the target enclave.
            if (destReactor.actions.size() != 1) {
              throw new RuntimeException("Found ConnectionReactor with number of actions != 1");
            }
            TimeValue delay = destReactor.actions.get(0).getMinDelay();
            boolean hasAfterDelay =
                ((CodeExprImpl)
                        destReactor
                            .getParameter("has_after_delay")
                            .getActualValue()
                            .getExprs()
                            .get(0))
                    .getCode()
                    .getBody()
                    .equals("true");
            addEdge(enclave, destEnclave, delay, hasAfterDelay, false);
          }
        }
      }
    }
  }

  /**
   * To find zero delay cycles in the enclave graph. We do a Depth First Search from each node and
   * look for backedges. However, since we are interested in zero-delay cycles. We only consider
   * edges without after delay. Edges with 'after 0' introduce a microstep delay.
   */
  public boolean hasZeroDelayCycles() {
    Set<ReactorInstance> visited = new HashSet<>();
    for (ReactorInstance node : graph.keySet()) {
      if (hasZeroDelayCycle(node, visited, new Stack<>())) {
        return true;
      }
    }
    return false;
  }

  /**
   * Perform the DFS
   *
   * @param current The node to search from.
   * @param visited The set of already visited nodes.
   * @param path The path till the current node.
   * @return If a cylce was found.
   */
  private boolean hasZeroDelayCycle(
      ReactorInstance current, Set<ReactorInstance> visited, Stack<ReactorInstance> path) {
    visited.add(current);
    path.push(current);

    // Loop through all zero-delay outgoing edges.
    for (EnclaveConnection edge :
        graph.get(current).stream().filter(e -> !e.hasAfterDelay()).toList()) {
      if (!visited.contains(edge.target)) {
        if (hasZeroDelayCycle(edge.target, visited, path)) {
          return true;
        }
      } else if (path.contains(edge.target)) {
        zeroDelayCycle = path;
        return true;
      }
    }

    path.pop();
    visited.remove(current);
    return false;
  }

  /**
   * Given an enclave. Return the "user enclave" which is contained inside the wrapper enclave. If
   * this is the main reactor, then there is no wrapper.
   *
   * @param enclave The enclave
   * @return The user enclave.
   */
  public ReactorInstance getUserEnclave(ReactorInstance enclave) {
    if (enclave.isMainOrFederated()) {
      return enclave;
    } else {
      return enclave.children.get(0);
    }
  }

  /**
   * If a zero-delay cycle is found and stored in the `cycle` field. Create a string containing the
   * cycle. Use the names of the user-enclaves not the wrapper enclaves.
   *
   * @return The string representing the cycle.
   */
  public String buildCycleString() {
    StringBuilder cycle = new StringBuilder();
    ReactorInstance start = getUserEnclave(zeroDelayCycle.get(0));
    for (ReactorInstance node : zeroDelayCycle) {
      cycle.append(getUserEnclave(node).getFullName()).append(" -> ");
    }
    cycle.append(start.getFullName());
    return cycle.toString();
  }

  /**
   * Add a single enclave as a node in the graph.
   *
   * @param node The enclave to add.
   */
  public void addNode(ReactorInstance node) {
    if (!graph.containsKey(node)) {
      graph.put(node, new HashSet<>());
    }
  }

  /**
   * Add a list of enclaves as nodes in the graph.
   *
   * @param nodes A list of enclaves to add.
   */
  public void addNodes(List<ReactorInstance> nodes) {
    for (ReactorInstance node : nodes) {
      addNode(node);
    }
  }

  /**
   * Add an edge between two existing nodes of the graph.
   *
   * @param source The source enclave of the edge.
   * @param target The target enclave of the edge.
   * @param delay The delay on the connection between source and target.
   * @param hasAfterDelay If the connection had an explicit after delay.
   * @param isPhysical If the connection was physical.
   */
  public void addEdge(
      ReactorInstance source,
      ReactorInstance target,
      TimeValue delay,
      boolean hasAfterDelay,
      boolean isPhysical) {
    if (graph.containsKey(source) && graph.containsKey(target)) {
      EnclaveConnection newEdge =
          new EnclaveConnection(source, target, delay, hasAfterDelay, isPhysical);
      graph.get(source).add(newEdge);
    } else {
      throw new RuntimeException("Tried adding edges between non-existing nodes.");
    }
  }

  /**
   * Returns the set of enclaves directly upstream of an enclave.
   *
   * @param node The enclave from which to search for upstreams.
   * @return The set of upstreams.
   */
  public Set<EnclaveConnection> getDirectUpstreams(ReactorInstance node) {
    Set<EnclaveConnection> upstreams = new HashSet<>();
    for (Map.Entry<ReactorInstance, Set<EnclaveConnection>> entry : graph.entrySet()) {
      for (EnclaveConnection edge : entry.getValue()) {
        if (edge.target().equals(node)) {
          upstreams.add(edge);
          break;
        }
      }
    }
    return upstreams;
  }

  /**
   * Returns the set of enclaves directly downstream of an enclave.
   *
   * @param node The enclave from which to look for downstreams.
   * @return The set of downstreams.
   */
  public Set<EnclaveConnection> getDirectDownstreams(ReactorInstance node) {
    return graph.getOrDefault(node, new HashSet<>());
  }
}
