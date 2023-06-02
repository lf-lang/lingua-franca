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

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.Stack;
import org.eclipse.xtext.xbase.lib.ListExtensions;

/**
 * Elaboration of {@code DirectedGraph} that is capable of identifying strongly connected components
 * and topologically sorting its nodes.
 *
 * @author Marten Lohstroh
 */
public class PrecedenceGraph<T extends Object> extends DirectedGraph<T> {

  /** Annotations used during the execution of Tarjan's algorithm. */
  private NodeAnnotations<T> annotations = new NodeAnnotations<T>();

  /**
   * Indicates whether or not the graph has been analyzed for cycles. If this variable is false,
   * Tarjan's algorithm need to be ran to find out whether or not this graph has cycles.
   */
  private boolean cycleAnalysisDone = false;

  /**
   * Indicates whether or not the graph has been sorted. If this variable is false, a new DFS has to
   * be done to compute a topological sort.
   */
  private boolean isSorted = false;

  /** Index used in Tarjan's algorithm. */
  private int index = 0;

  /** After analysis has completed, this list contains all nodes in reverse topological order. */
  private List<T> sortedNodes = new ArrayList<>();

  /** Stack used in Tarjan's algorithm. */
  private Stack<T> stack = new Stack<>();

  /**
   * After analysis has completed, this list contains all all sets of nodes that are part of the
   * same strongly connected component.
   */
  protected List<Set<T>> cycles = new ArrayList<>();

  /** Invalidate cached analysis due to changes in the graph structure. */
  @Override
  public void graphChanged() {
    this.cycleAnalysisDone = false;
    this.isSorted = false;
  }

  /** Construct a new dependency graph. */
  public PrecedenceGraph() {}

  /** Topologically sort the nodes in the graph. */
  private void sortNodes() {
    if (!this.isSorted) {
      // Cleanup.
      this.sortedNodes = new ArrayList<>();
      this.nodes()
          .forEach(
              it -> {
                this.annotations.get(it).hasTempMark = false;
                this.annotations.get(it).hasPermMark = false;
              });

      // Start sorting.
      for (T node : this.nodes()) {
        if (!this.annotations.get(node).hasPermMark) {
          // Unmarked node.
          this.visit(node);
        }
      }
      this.isSorted = true;
    }
  }

  /**
   * Recursively visit all nodes reachable from the given node; after all those nodes have been
   * visited add the current node to a list which will be sorted in reverse order.
   */
  private void visit(T node) {
    NodeAnnotation annotation = this.annotations.get(node);
    if (annotation.hasPermMark) {
      return;
    }
    if (annotation.hasTempMark) {
      // Not a DAG.
      throw new Error("Cannot order nodes due to cycle in the graph.");
    }
    annotation.hasTempMark = true;
    for (T dep : this.getDownstreamAdjacentNodes(node)) {
      visit(dep);
    }
    annotation.hasTempMark = false;
    annotation.hasPermMark = true;
    this.sortedNodes.add(node);
  }

  /**
   * Run Tarjan's algorithm for finding strongly connected components. After invoking this method,
   * the detected cycles with be listed in the class variable {@code cycles}.
   */
  public void detectCycles() {
    if (!this.cycleAnalysisDone) {
      this.index = 0;
      this.stack = new Stack<>();
      this.cycles = new ArrayList<>();
      this.nodes()
          .forEach(
              it -> {
                this.annotations.get(it).index = -1;
              });
      for (T node : this.nodes()) {
        if (this.annotations.get(node).index == -1) {
          this.strongConnect(node);
        }
      }
      this.cycleAnalysisDone = true;
      stack = null;
    }
  }

  /** Report whether this graph has any cycles in it. */
  public boolean hasCycles() {
    this.detectCycles();
    return this.cycles.size() > 0;
  }

  /** Return a list of strongly connected components that exist in this graph. */
  public List<Set<T>> getCycles() {
    this.detectCycles();
    return this.cycles;
  }

  /**
   * Traverse the graph to visit unvisited dependencies and determine whether they are part of a
   * cycle.
   */
  public void strongConnect(T node) {
    NodeAnnotation annotation = this.annotations.get(node);
    annotation.index = this.index;
    annotation.lowLink = this.index;
    annotation.onStack = true;
    this.index++;
    this.stack.push(node);
    for (T dep : this.getUpstreamAdjacentNodes(node)) {
      NodeAnnotation depAnnotation = this.annotations.get(dep);
      if (depAnnotation.onStack) {
        annotation.lowLink = Math.min(annotation.lowLink, depAnnotation.index);
        if (node.equals(dep)) {
          annotation.selfLoop = true;
        }
      } else if (depAnnotation.index == -1) {
        strongConnect(dep);
        annotation.lowLink = Math.min(annotation.lowLink, depAnnotation.lowLink);
      }
    }

    if (annotation.lowLink == annotation.index) {
      Set<T> scc = new LinkedHashSet<>();
      T dep = null;
      do {
        dep = this.stack.pop();
        this.annotations.get(dep).onStack = false;
        scc.add(dep);
      } while (!node.equals(dep));
      // Only report self loops or cycles with two or more nodes.
      if (scc.size() > 1 || annotation.selfLoop) {
        this.cycles.add(scc);
      }
    }
  }

  /**
   * Return the nodes of this graph in reverse topological order. Each node in the returned list is
   * succeeded by the nodes that it depends on.
   */
  public List<T> nodesInReverseTopologicalOrder() {
    this.sortNodes();
    return this.sortedNodes;
  }

  /**
   * Return the nodes of this graph in reverse topological order. Each node in the returned list is
   * preceded by the nodes that it depends on.
   */
  public List<T> nodesInTopologicalOrder() {
    this.sortNodes();
    return ListExtensions.reverse(this.sortedNodes);
  }
}
