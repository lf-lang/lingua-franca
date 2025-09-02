package org.lflang.graph;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;

/**
 * This implements a special graph that I call a ConnectionGraph. It differs from the other graphs
 * in that both nodes and edges are parameterizable. So we can annotate nodes and edges. This is
 * needed for the connection graphs because we need to annotate an edge with the delay and whether
 * it is physical or not.
 */
public class ConnectionGraph<V extends Object, E extends Object> {

  private final Map<V, Map<V, Set<E>>> graph = new HashMap<>();

  public void addEdge(V source, V destination, E edge) {
    Map<V, Set<E>> destMap = graph.getOrDefault(source, new HashMap<>());
    Set<E> edges = destMap.getOrDefault(destination, new HashSet<>());
    edges.add(edge);
    destMap.put(destination, edges);
    graph.put(source, destMap);
  }

  public void addEdges(V source, V destination, Set<E> edges) {
    Map<V, Set<E>> destMap = graph.getOrDefault(source, new HashMap<>());
    Set<E> edgesOriginal = destMap.getOrDefault(destination, new HashSet<>());
    edgesOriginal.addAll(edges);
    destMap.put(destination, edgesOriginal);
    graph.put(source, destMap);
  }

  public Map<V, Set<E>> getUpstreamOf(V node) {
    Map<V, Set<E>> res = new HashMap<>();
    for (V el : graph.keySet()) {
      if (!node.equals(el)) {
        for (V dest : graph.get(el).keySet()) {
          if (dest != null && dest.equals(node)) {
            Set<E> destSet = res.getOrDefault(dest, new HashSet<>());
            destSet.addAll(graph.get(el).get(dest));
            res.put(el, destSet);
          }
        }
      }
    }
    return res;
  }

  public Map<V, Set<E>> getDownstreamOf(V node) {
    return graph.getOrDefault(node, new HashMap<>());
  }

  public List<V> getNodes() {
    return graph.keySet().stream().toList();
  }

  public ConnectionGraph() {}
}
