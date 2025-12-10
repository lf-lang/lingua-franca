package org.lflang.generator.c;

import static org.lflang.AttributeUtils.isEnclave;

import java.util.HashSet;
import java.util.LinkedList;
import java.util.Queue;
import java.util.Set;
import java.util.Stack;
import org.lflang.TimeValue;
import org.lflang.generator.ReactorInstance;
import org.lflang.graph.ConnectionGraph;
import org.lflang.lf.Instantiation;

/**
 * This class contains the enclave connection graph. This graph is needed to code-generate the
 * topology information in the generated sources. To simplify things we use the AST transformation
 * object. It stores information about the topology as it is doing the AST transformation.
 *
 * @ingroup Generator
 */
public class CEnclaveGraph {
  /** The graph. Public to expose its API to the code generator also. */
  public ConnectionGraph<CEnclaveInstance, EnclaveConnection> graph = new ConnectionGraph<>();

  /** The AST transformation. It will store topology information. */
  private final CEnclavedReactorTransformation ast;

  private Stack<CEnclaveInstance> zeroDelayCycle = new Stack<>();

  public CEnclaveGraph(CEnclavedReactorTransformation ast) {
    this.ast = ast;
  }

  /**
   * @brief A record to store the information about an enclave connection.
   * @param delay The delay of the connection.
   * @param hasAfterDelay Whether the connection has an after delay.
   * @param isPhysical Whether the connection is physical.
   */
  public record EnclaveConnection(TimeValue delay, boolean hasAfterDelay, boolean isPhysical) {}

  /**
   * @brief Build the enclave graph.
   *     <p>This function takes the main reactor instance and the set of enclave instances and does
   *     a breadth-first search. When a reactor instance which is annotated with {@code @enclave} is found,
   *     we check the AST transformation for what instantiations are connected to this reactor. We
   *     then look through the other reactor instances at the same level and find the corresponding
   *     reactor instance. Understanding this requires that you understand the difference between
   *     the Reactor Instances and Reactor Instantiations. The AST transformation knows how the
   *     Reactor Instantiations are connected, but we must translate it to how Reactor Instances are
   *     connected.
   * @param main The main reactor, top-level enclave.
   * @param enclaves The set of enclaves to build the graph for.
   */
  public void build(ReactorInstance main, Set<CEnclaveInstance> enclaves) {
    Queue<ReactorInstance> queue = new LinkedList<>();
    queue.add(main);
    while (!queue.isEmpty()) {
      ReactorInstance current = queue.poll();
      for (ReactorInstance child : current.children) {
        Instantiation inst = child.getDefinition();
        if (isEnclave(inst)) {
          // Get the Enclave instance associated with the reactor instance
          CEnclaveInstance enc = child.containingEnclave;
          // Get all instantiations that are downstream of the instantitation associated with the
          // reactor instance.
          var downStreamMap = ast.connGraph.getDownstreamOf(inst);
          for (Instantiation downstream : downStreamMap.keySet()) {
            // For each downstream instantiation. Find the corresponding ReactorInstance
            if (downstream.equals(ast.PARENT)) {
              // We have a special instantiation which represents that the enclave is connected to
              // its parent.
              graph.addEdges(enc, current.containingEnclave, downStreamMap.get(downstream));
            } else {
              // Find the reactor instance which is connected to this instantiation.
              ReactorInstance down =
                  current.children.stream()
                      .filter(i -> i.getDefinition() == downstream)
                      .findFirst()
                      .get();
              CEnclaveInstance downEnclave = down.containingEnclave;
              graph.addEdges(enc, downEnclave, downStreamMap.get(downstream));
            }
          }
          // Redo the steps to find upstream reactor instances. Note that adding both upstream
          // and downstream edges will lead to duplicates. But these are ignored by the underlying
          // graph implementation.
          var upstreamMap = ast.connGraph.getUpstreamOf(inst);
          for (Instantiation upstream : upstreamMap.keySet()) {
            if (upstream.equals(ast.PARENT)) {
              // If upstream was `null` then we have a connection to the parent.
              graph.addEdges(current.containingEnclave, enc, upstreamMap.get(upstream));
            } else {
              ReactorInstance up =
                  current.children.stream()
                      .filter(i -> i.getDefinition() == upstream)
                      .findFirst()
                      .get();
              graph.addEdges(up.containingEnclave, enc, upstreamMap.get(upstream));
            }
          }
        } else if (ast.enclavedConnections.containsKey(inst)) {
          // inst is the instantiation of the enclave connection reactor.
          // Now we have enough information to set the source_env and dest_env parameters
          // of the generated EnclavedConnection reactors.
          ReactorInstance source = null;
          ReactorInstance dest = null;
          // Get the upstream and downstream instantiations for this connection reactor
          var envs = ast.enclavedConnections.get(inst);
          // First check whether any of the instantiations are actually the parent.
          if (envs.first().equals(ast.PARENT)) {
            source = current;
          }

          if (envs.second().equals(ast.PARENT)) {
            dest = current;
          }
          // Search the other reactor instances.
          for (ReactorInstance c : current.children) {
            if (envs.first().equals(c.getDefinition())) {
              source = c;
            }
            if (envs.second().equals(c.getDefinition())) {
              dest = c;
            }
          }
          // Update the environment parameters.
          ast.setEnvParams(child, source, dest);
        }
        queue.add(child);
      }
    }
  }

  /**
   * Return whether the enclave graph has a zero-delay cycle. To find zero delay cycles in the
   * enclave graph. We do a Depth First Search from each node and look for backedges. However, since
   * we are interested in zero-delay cycles. We only consider edges without after delay. Edges with
   * 'after 0' introduce a microstep delay.
   */
  public boolean hasZeroDelayCycle() {
    Set<CEnclaveInstance> visited = new HashSet<>();
    for (CEnclaveInstance node : graph.getNodes()) {
      if (_hasZeroDelayCycle(node, visited, new Stack<>())) {
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
  private boolean _hasZeroDelayCycle(
      CEnclaveInstance current, Set<CEnclaveInstance> visited, Stack<CEnclaveInstance> path) {
    visited.add(current);
    path.push(current);

    var downstreams = graph.getDownstreamOf(current);
    for (CEnclaveInstance downstream : downstreams.keySet()) {
      if (downstreams.get(downstream).stream().anyMatch(c -> !c.hasAfterDelay())) {
        if (!visited.contains(downstream)) {
          if (_hasZeroDelayCycle(downstream, visited, path)) {
            return true;
          }
        } else if (path.contains(downstream)) {
          zeroDelayCycle = path;
          return true;
        }
      }
    }
    path.pop();
    visited.remove(current);
    return false;
  }

  /**
   * If a zero-delay cycle is found and stored in the `zeroDelayCycle` field. Create a string
   * containing the cycle. To be printed to the user.
   *
   * @return The string representing the cycle.
   */
  public String buildCycleString() {
    StringBuilder cycle = new StringBuilder();
    CEnclaveInstance start = zeroDelayCycle.get(0);
    for (CEnclaveInstance node : zeroDelayCycle) {
      cycle.append(node.getReactorInstance().getFullName()).append(" -> ");
    }
    cycle.append(start.getReactorInstance().getFullName());
    return cycle.toString();
  }
}
