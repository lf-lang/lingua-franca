/*************
 * Copyright (c) 2019, The University of California at Berkeley.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.graph;

import java.util.*;
import java.util.Map.Entry;
import java.util.stream.Collectors;
import org.lflang.util.CollectionUtil;

/**
 * Directed graph that maps nodes to its upstream and downstream neighbors.
 *
 * @author Marten Lohstroh
 * @author Clément Fournier
 */
public class DirectedGraph<T> implements Graph<T> {

  // Note that while both those maps are mutable, the sets
  // they use as values may not be. They should only be
  // manipulated through CollectionUtil

  // If a node has no neighbors, it is still in the map with an empty set as a value.

  /** Adjacency map from vertices to their downstream immediate neighbors. */
  private final Map<T, Set<T>> downstreamAdjacentNodes = new LinkedHashMap<>();

  /** Adjacency map from vertices to their upstream immediate neighbors. */
  private final Map<T, Set<T>> upstreamAdjacentNodes = new LinkedHashMap<>();

  /** Mark the graph to have changed so that any cached analysis is refreshed accordingly. */
  protected void graphChanged() {
    // To be overridden by subclasses that perform analysis.
  }

  /**
   * Return true if this graph has the given node in it.
   *
   * @param node The node to look for.
   */
  @Override
  public boolean hasNode(T node) {
    return nodes().contains(node);
  }

  /**
   * Return all immediate upstream neighbors of a given node.
   *
   * @param node The node to report the immediate upstream neighbors of.
   */
  public Set<T> getUpstreamAdjacentNodes(T node) {
    return Collections.unmodifiableSet(this.upstreamAdjacentNodes.getOrDefault(node, Set.of()));
  }

  /**
   * Return all immediate downstream neighbors of a given node.
   *
   * @param node The node to report the immediate downstream neighbors of.
   */
  public Set<T> getDownstreamAdjacentNodes(T node) {
    return Collections.unmodifiableSet(this.downstreamAdjacentNodes.getOrDefault(node, Set.of()));
  }

  @Override
  public void addNode(T node) {
    this.graphChanged();
    this.upstreamAdjacentNodes.putIfAbsent(node, Set.of());
    this.downstreamAdjacentNodes.putIfAbsent(node, Set.of());
  }

  @Override
  public void removeNode(T node) {
    this.graphChanged();
    this.upstreamAdjacentNodes.remove(node);
    this.downstreamAdjacentNodes.remove(node);
    // The node also needs to be removed from the sets that represent connections to the node.
    CollectionUtil.removeFromValues(this.upstreamAdjacentNodes, node);
    CollectionUtil.removeFromValues(this.downstreamAdjacentNodes, node);
  }

  /**
   * Add a new directed edge to the graph. The first argument is the downstream node, the second
   * argument the upstream node. If either argument is null, do nothing.
   *
   * @param sink The downstream immediate neighbor.
   * @param source The upstream immediate neighbor.
   */
  @Override
  public void addEdge(T sink, T source) {
    this.graphChanged();
    if (sink != null && source != null) {
      this.downstreamAdjacentNodes.compute(source, (k, set) -> CollectionUtil.plus(set, sink));
      this.upstreamAdjacentNodes.compute(sink, (k, set) -> CollectionUtil.plus(set, source));
    }
  }

  /**
   * Add new directed edges to the graph. The first argument is the downstream node, the second
   * argument a set of upstream nodes.
   *
   * @param sink The downstream immediate neighbor.
   * @param sources The upstream immediate neighbors.
   */
  @Override
  public void addEdges(T sink, List<T> sources) {
    for (T source : sources) {
      this.addEdge(sink, source);
    }
  }

  /**
   * Remove a directed edge from the graph.
   *
   * @param sink The downstream immediate neighbor.
   * @param source The upstream immediate neighbor.
   */
  @Override
  public void removeEdge(T sink, T source) {
    this.graphChanged();
    this.upstreamAdjacentNodes.computeIfPresent(
        sink, (k, upstream) -> CollectionUtil.minus(upstream, source));
    this.downstreamAdjacentNodes.computeIfPresent(
        source, (k, downstream) -> CollectionUtil.minus(downstream, sink));
  }

  /** Obtain a copy of this graph by creating an new instance and copying the adjacency maps. */
  public DirectedGraph<T> copy() {
    var graph = new DirectedGraph<T>();
    for (var entry : this.upstreamAdjacentNodes.entrySet()) {
      graph.upstreamAdjacentNodes.put(entry.getKey(), CollectionUtil.copy(entry.getValue()));
    }
    for (var entry : this.downstreamAdjacentNodes.entrySet()) {
      graph.downstreamAdjacentNodes.put(entry.getKey(), CollectionUtil.copy(entry.getValue()));
    }
    return graph;
  }

  /**
   * For a given a two adjacency maps, copy missing edges from the first map to the second.
   *
   * @param srcMap The adjacency map to copy edges from.
   * @param dstMap The adjacency map to copy edges to.
   */
  private void mirror(Map<T, Set<T>> srcMap, Map<T, Set<T>> dstMap) {
    if (srcMap != null && dstMap != null) {
      for (Entry<T, Set<T>> entry : srcMap.entrySet()) {
        var node = entry.getKey();
        var srcEdges = entry.getValue();
        dstMap.compute(
            node,
            (_node, dstEdges) -> {
              // Node does not exist; add it.
              if (dstEdges == null) {
                return CollectionUtil.copy(srcEdges);
              }

              // Node does exist; add the missing edges.
              var set = dstEdges;
              for (T edge : srcEdges) {
                set = CollectionUtil.plus(set, edge);
              }
              return set;
            });
      }
    }
  }

  /**
   * Merge another directed graph into this one.
   *
   * @param another The graph to merge into this one.
   */
  public void merge(DirectedGraph<T> another) {
    this.graphChanged();
    mirror(another.upstreamAdjacentNodes, this.upstreamAdjacentNodes);
    mirror(another.downstreamAdjacentNodes, this.downstreamAdjacentNodes);
  }

  /** Return the set of nodes that have no neighbors listed in the given adjacency map. */
  private Set<T> independentNodes(Map<T, Set<T>> adjacencyMap) {
    var independent = new LinkedHashSet<T>();
    for (T node : nodes()) {
      var neighbors = adjacencyMap.get(node);
      if (neighbors == null || neighbors.size() == 0) {
        independent.add(node);
      }
    }
    return independent;
  }

  /** Return the root nodes of this graph. Root nodes have no upstream neighbors. */
  public Set<T> rootNodes() {
    return independentNodes(this.upstreamAdjacentNodes);
  }

  /** Return the leaf nodes of this graph. Leaf nodes have no downstream neighbors. */
  public Set<T> leafNodes() {
    return independentNodes(this.downstreamAdjacentNodes);
  }

  @Override
  public int nodeCount() {
    return downstreamAdjacentNodes.size();
  }

  @Override
  public int edgeCount() {
    return this.upstreamAdjacentNodes.values().stream().mapToInt(Set::size).sum();
  }

  @Override
  public Set<T> nodes() {
    return Collections.unmodifiableSet(this.downstreamAdjacentNodes.keySet());
  }

  public void clear() {
    this.graphChanged();
    this.downstreamAdjacentNodes.clear();
    this.upstreamAdjacentNodes.clear();
  }

  /** Return a textual list of the nodes. */
  @Override
  public String toString() {
    return nodes().stream().map(Objects::toString).collect(Collectors.joining(", ", "{", "}"));
  }

  /** Return the DOT (GraphViz) representation of the graph. */
  @Override
  public String toDOT() {
    StringBuilder dotRepresentation = new StringBuilder();
    StringBuilder edges = new StringBuilder();

    // Start the digraph with a left-write rank
    dotRepresentation.append("digraph {\n");
    dotRepresentation.append("    rankdir=LF;\n");

    Set<T> nodes = nodes();
    for (T node : nodes) {
      // Draw the node
      dotRepresentation.append(
          "    node_"
              + (node.toString().hashCode() & 0xfffffff)
              + " [label=\""
              + node.toString()
              + "\"]\n");

      // Draw the edges
      Set<T> downstreamNodes = getDownstreamAdjacentNodes(node);
      for (T downstreamNode : downstreamNodes) {
        edges.append(
            "    node_"
                + (node.toString().hashCode() & 0xfffffff)
                + " -> node_"
                + (downstreamNode.toString().hashCode() & 0xfffffff)
                + "\n");
      }
    }

    // Add the edges to the definition of the graph at the bottom
    dotRepresentation.append(edges);

    // Close the digraph
    dotRepresentation.append("}\n");

    // Return the DOT representation
    return dotRepresentation.toString();
  }
}
