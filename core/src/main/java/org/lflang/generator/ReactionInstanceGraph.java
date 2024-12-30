/*************
 * Copyright (c) 2021, The University of California at Berkeley.
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

package org.lflang.generator;

import java.util.*;
import java.util.stream.Collectors;
import org.lflang.generator.ReactionInstance.Runtime;
import org.lflang.generator.c.CUtil;
import org.lflang.graph.PrecedenceGraph;
import org.lflang.lf.Variable;

/**
 * This class analyzes the dependencies between reaction runtime instances. For each
 * ReactionInstance, there may be more than one runtime instance because the ReactionInstance may be
 * nested within one or more banks. In the worst case, of these runtime instances may have distinct
 * dependencies, and hence distinct levels in the graph. Moreover, some of these instances may be
 * involved in cycles while others are not.
 *
 * <p>Upon construction of this class, the runtime instances are created if necessary, stored each
 * ReactionInstance, and assigned levels (maximum number of upstream reaction instances), deadlines,
 * and single dominating reactions.
 *
 * <p>After creation, the resulting graph will be empty unless there are causality cycles, in which
 * case, the resulting graph is a graph of runtime reaction instances that form cycles.
 *
 * @author Marten Lohstroh
 * @author Edward A. Lee
 */
public class ReactionInstanceGraph extends PrecedenceGraph<ReactionInstance.Runtime> {

  /**
   * Create a new graph by traversing the maps in the named instances embedded in the hierarchy of
   * the program.
   */
  public ReactionInstanceGraph(ReactorInstance main) {
    this.main = main;
    rebuild();
  }

  ///////////////////////////////////////////////////////////
  //// Public fields

  /** The main reactor instance that this graph is associated with. */
  public final ReactorInstance main;

  /**
   * Rebuild this graph by clearing and repeating the traversal that adds all the nodes and edges.
   * Note that after this executes, the graph is empty unless it has causality cycles, in which case
   * it contains only those causality cycles.
   */
  private void rebuild() {
    this.clear();
    addNodesAndEdges(main);
    addEdgesForTpoLevels(main);
    assignInferredDeadlines();

    // Assign a level to each reaction.
    // If there are cycles present in the graph, it will be detected here.
    // This will destroy the graph, leaving only nodes in cycles.
    assignLevels();
    // Do not throw an exception when nodeCount != 0 so that cycle visualization can proceed.
  }

  /*
   * Get an array of non-negative integers representing the number of reactions
   * per each level, where levels are indices of the array.
   */
  public Integer[] getNumReactionsPerLevel() {
    return numReactionsPerLevel.toArray(new Integer[0]);
  }

  /** Return the max breadth of the reaction dependency graph */
  public int getBreadth() {
    var maxBreadth = 0;
    for (Integer breadth : numReactionsPerLevel) {
      if (breadth > maxBreadth) {
        maxBreadth = breadth;
      }
    }
    return maxBreadth;
  }

  ///////////////////////////////////////////////////////////
  //// Protected methods

  /**
   * Add to the graph edges between the given reaction and all the reactions that depend on the
   * specified port.
   *
   * @param port The port that the given reaction has as an effect.
   * @param reaction The reaction to relate downstream reactions to.
   */
  protected void addDownstreamReactions(PortInstance port, ReactionInstance reaction) {
    // Use mixed-radix numbers to increment over the ranges.
    List<Runtime> srcRuntimes = reaction.getRuntimeInstances();
    List<SendRange> eventualDestinations = port.eventualDestinations();

    int srcDepth = (port.isInput()) ? 2 : 1;

    for (SendRange sendRange : eventualDestinations) {
      for (RuntimeRange<PortInstance> dstRange : sendRange.destinations) {

        int dstDepth = (dstRange.instance.isOutput()) ? 2 : 1;
        MixedRadixInt dstRangePosition = dstRange.startMR();
        int dstRangeCount = 0;

        MixedRadixInt sendRangePosition = sendRange.startMR();
        int sendRangeCount = 0;

        while (dstRangeCount++ < dstRange.width) {
          int srcIndex = sendRangePosition.get(srcDepth);
          int dstIndex = dstRangePosition.get(dstDepth);
          for (ReactionInstance dstReaction : dstRange.instance.dependentReactions) {
            List<Runtime> dstRuntimes = dstReaction.getRuntimeInstances();
            Runtime srcRuntime = srcRuntimes.get(srcIndex);
            Runtime dstRuntime = dstRuntimes.get(dstIndex);
            // Only add this dependency if the reactions are not in modes at all or in the same mode
            // or in modes of separate reactors
            // This allows modes to break cycles since modes are always mutually exclusive.
            if (srcRuntime.getReaction().getMode(true) == null
                || dstRuntime.getReaction().getMode(true) == null
                || srcRuntime.getReaction().getMode(true) == dstRuntime.getReaction().getMode(true)
                || srcRuntime.getReaction().getParent() != dstRuntime.getReaction().getParent()) {
              addEdge(dstRuntime, srcRuntime);
            }

            // Propagate the deadlines, if any.
            if (srcRuntime.deadline.compareTo(dstRuntime.deadline) > 0) {
              srcRuntime.deadline = dstRuntime.deadline;
            }

            // If this seems to be a single dominating reaction, set it.
            // If another upstream reaction shows up, then this will be
            // reset to null.
            if (this.getUpstreamAdjacentNodes(dstRuntime).size() == 1
                && (dstRuntime.getReaction().index == 0)) {
              dstRuntime.dominating = srcRuntime;
            } else {
              dstRuntime.dominating = null;
            }
          }
          dstRangePosition.increment();
          sendRangePosition.increment();
          sendRangeCount++;
          if (sendRangeCount >= sendRange.width) {
            // Reset to multicast.
            sendRangeCount = 0;
            sendRangePosition = sendRange.startMR();
          }
        }
      }
    }
  }

  /**
   * Build the graph by adding nodes and edges based on the given reactor instance.
   *
   * @param reactor The reactor on the basis of which to add nodes and edges.
   */
  protected void addNodesAndEdges(ReactorInstance reactor) {
    ReactionInstance previousReaction = null;
    for (ReactionInstance reaction : reactor.reactions) {
      List<Runtime> runtimes = reaction.getRuntimeInstances();

      // Add reactions of this reactor.
      for (Runtime runtime : runtimes) {
        this.addNode(runtime);
      }

      // If there is an earlier reaction in this same reactor, then
      // create a link in the reaction graph for all runtime instances.
      if (previousReaction != null) {
        List<Runtime> previousRuntimes = previousReaction.getRuntimeInstances();
        int count = 0;
        for (Runtime runtime : runtimes) {
          // Only add the reaction order edge if previous reaction is outside of a mode or both are
          // in the same mode
          // This allows modes to break cycles since modes are always mutually exclusive.
          if (runtime.getReaction().getMode(true) == null
              || runtime.getReaction().getMode(true) == reaction.getMode(true)) {
            this.addEdge(runtime, previousRuntimes.get(count));
            count++;
          }
        }
      }
      previousReaction = reaction;

      // Add downstream reactions. Note that this is sufficient.
      // We don't need to also add upstream reactions because this reaction
      // will be downstream of those upstream reactions.
      for (TriggerInstance<? extends Variable> effect : reaction.effects) {
        if (effect instanceof PortInstance) {
          addDownstreamReactions((PortInstance) effect, reaction);
        }
      }
    }
    // Recursively add nodes and edges from contained reactors.
    for (ReactorInstance child : reactor.children) {
      addNodesAndEdges(child);
    }
    registerPortInstances(reactor);
  }

  /**
   * Add edges that encode the precedence relations induced by the TPO levels. TPO is total port
   * order. See
   * https://github.com/icyphy/lf-pubs/blob/54af48a97cc95058dbfb3333b427efb70294f66c/federated/TOMACS/paper.tex#L1353
   */
  private void addEdgesForTpoLevels(ReactorInstance main) {
    var constrainedReactions = getConstrainedReactions(main);
    for (var i : constrainedReactions.keySet()) {
      var nextKey = constrainedReactions.higherKey(i);
      if (nextKey == null) continue;
      for (var r : constrainedReactions.get(i)) {
        for (var rr : constrainedReactions.get(nextKey)) {
          addEdge(rr, r);
        }
      }
    }
  }

  /**
   * Get those reactions contained directly or transitively by the children of {@code main} whose
   * TPO levels are specified.
   *
   * @return A map from TPO levels to reactions that are constrained to have the TPO levels.
   */
  private NavigableMap<Integer, List<Runtime>> getConstrainedReactions(ReactorInstance main) {
    NavigableMap<Integer, List<Runtime>> constrainedReactions = new TreeMap<>();
    for (var child : main.children) {
      if (child.tpoLevel != null) {
        if (!constrainedReactions.containsKey(child.tpoLevel)) {
          constrainedReactions.put(child.tpoLevel, new ArrayList<>());
        }
        getAllContainedReactions(constrainedReactions.get(child.tpoLevel), child);
      }
    }
    return constrainedReactions;
  }

  /** Add all reactions contained directly or transitively by {@code r}. */
  private void getAllContainedReactions(List<Runtime> runtimeReactions, ReactorInstance r) {
    for (var reaction : r.reactions) {
      runtimeReactions.addAll(reaction.getRuntimeInstances());
    }
    for (var child : r.children) getAllContainedReactions(runtimeReactions, child);
  }

  ///////////////////////////////////////////////////////////
  //// Private fields

  /**
   * Number of reactions per level, represented as a list of integers where the indices are the
   * levels.
   */
  private final List<Integer> numReactionsPerLevel = new ArrayList<>(List.of(0));

  ///////////////////////////////////////////////////////////
  //// Private methods

  /** A port and an index of a reaction relative to the port. */
  public record MriPortPair(MixedRadixInt index, PortInstance port) {}

  /**
   * For each port in {@code reactor}, add that port to its downstream reactions, together with the
   * {@code MixedRadixInt} that is the index of the downstream reaction relative to the port and the
   * intervening ports.
   */
  private void registerPortInstances(ReactorInstance reactor) {
    var allPorts = new ArrayList<PortInstance>();
    allPorts.addAll(reactor.inputs);
    allPorts.addAll(reactor.outputs);
    for (var port : allPorts) {
      List<SendRange> eventualDestinations = port.eventualDestinations();

      for (SendRange sendRange : eventualDestinations) {
        for (RuntimeRange<PortInstance> dstRange : sendRange.destinations) {

          int dstDepth = (dstRange.instance.isOutput()) ? 2 : 1;
          MixedRadixInt dstRangePosition = dstRange.startMR();
          int dstRangeCount = 0;

          MixedRadixInt sendRangePosition = sendRange.startMR();
          int sendRangeCount = 0;

          while (dstRangeCount++ < dstRange.width) {
            int dstIndex = dstRangePosition.get(dstDepth);
            for (ReactionInstance dstReaction : dstRange.instance.dependentReactions) {
              List<Runtime> dstRuntimes = dstReaction.getRuntimeInstances();
              Runtime dstRuntime = dstRuntimes.get(dstIndex);
              dstRuntime.sourcePorts.add(new MriPortPair(sendRangePosition.copy(), port));
            }
            dstRangePosition.increment();
            sendRangePosition.increment();
            sendRangeCount++;
            if (sendRangeCount >= sendRange.width) {
              // Reset to multicast.
              sendRangeCount = 0;
              sendRangePosition = sendRange.startMR();
            }
          }
        }
      }
    }
  }

  /**
   * Analyze the dependencies between reactions and assign each reaction instance a level. This
   * method removes nodes from this graph as it assigns levels. Any remaining nodes are part of
   * causality cycles.
   *
   * <p>This procedure is based on Kahn's algorithm for topological sorting. Rather than
   * establishing a total order, we establish a partial order. In this order, the level of each
   * reaction is the least upper bound of the levels of the reactions it depends on.
   */
  private void assignLevels() {
    List<ReactionInstance.Runtime> start = new ArrayList<>(rootNodes());

    // All root nodes start with level 0.
    for (Runtime origin : start) {
      origin.level = 0;
    }

    // No need to do any of this if there are no root nodes;
    // the graph must be cyclic.
    while (!start.isEmpty()) {
      Runtime origin = start.remove(0);
      Set<Runtime> toRemove = new LinkedHashSet<>();
      Set<Runtime> downstreamAdjacentNodes = getDownstreamAdjacentNodes(origin);

      // Visit effect nodes.
      for (Runtime effect : downstreamAdjacentNodes) {
        // Stage edge between origin and effect for removal.
        toRemove.add(effect);

        // Update level of downstream node.
        effect.level = origin.level + 1;
      }
      // Remove visited edges.
      for (Runtime effect : toRemove) {
        removeEdge(effect, origin);
        // If the effect node has no more incoming edges,
        // then move it in the start set.
        if (getUpstreamAdjacentNodes(effect).isEmpty()) {
          start.add(effect);
        }
      }

      // Remove visited origin.
      removeNode(origin);
      assignPortLevel(origin);

      // Update numReactionsPerLevel info
      incrementNumReactionsPerLevel(origin.level);
    }
  }

  /**
   * Update the level of the source ports of {@code current} to be at most that of {@code current}.
   */
  private void assignPortLevel(Runtime current) {
    for (var sp : current.sourcePorts) {
      sp.port().hasDependentReactionWithLevel(sp.index(), current.level);
    }
  }

  /**
   * This function assigns inferred deadlines to all the reactions in the graph. It is modeled after
   * {@code assignLevels} but it starts at the leaf nodes, but it does not destroy the graph.
   */
  private void assignInferredDeadlines() {
    List<Runtime> start = new ArrayList<>(leafNodes());
    Set<Runtime> visited = new HashSet<>();

    // All leaf nodes have deadline initialized to their declared deadline or MAX_VALUE
    while (!start.isEmpty()) {
      Runtime origin = start.remove(0);
      visited.add(origin);
      Set<Runtime> upstreamAdjacentNodes = getUpstreamAdjacentNodes(origin);

      // Visit upstream nodes.
      for (Runtime upstream : upstreamAdjacentNodes) {
        // If the upstream node has been visited, then we have a cycle, which will be
        // an error condition. Skip it.
        if (visited.contains(upstream)) continue;
        // Update deadline of upstream node if origins deadline is earlier.
        if (origin.deadline.isEarlierThan(upstream.deadline)) {
          upstream.deadline = origin.deadline;
        }
        // Determine whether the upstream node is now a leaf node.
        var isLeaf = true;
        for (Runtime downstream : getDownstreamAdjacentNodes(upstream)) {
          if (!visited.contains(downstream)) isLeaf = false;
        }
        if (isLeaf) start.add(upstream);
      }
    }
  }

  /**
   * Adjust {@link #numReactionsPerLevel} at index <code>level<code> by
   * adding to the previously recorded number <code>valueToAdd<code>.
   * If there is no previously recorded number for this level, then
   * create one with index <code>level</code> and value <code>valueToAdd</code>.
   * @param level The level.
   */
  private void incrementNumReactionsPerLevel(int level) {
    if (numReactionsPerLevel.size() > level) {
      numReactionsPerLevel.set(level, numReactionsPerLevel.get(level) + 1);
    } else {
      while (numReactionsPerLevel.size() < level) {
        numReactionsPerLevel.add(0);
      }
      numReactionsPerLevel.add(1);
    }
  }

  /** Return the DOT (GraphViz) representation of the graph. */
  @Override
  public String toDOT() {
    var dotRepresentation = new CodeBuilder();
    var edges = new StringBuilder();

    // Start the digraph with a left-write rank
    dotRepresentation.pr(
        """
        digraph {
            rankdir=LF;
            graph [compound=True, rank=LR, rankdir=LR];
            node [fontname=Times, shape=rectangle];
            edge [fontname=Times];
        """);

    var nodes = nodes();
    // Group nodes by levels
    var groupedNodes = nodes.stream().collect(Collectors.groupingBy(it -> it.level));

    dotRepresentation.indent();
    // For each level
    for (var level : groupedNodes.keySet()) {
      // Create a subgraph
      dotRepresentation.pr("subgraph cluster_level_" + level + " {");
      dotRepresentation.pr("    graph [compound=True, label = \"level " + level + "\"];");

      // Get the nodes at the current level
      var currentLevelNodes = groupedNodes.get(level);
      for (var node : currentLevelNodes) {
        // Draw the node
        var label =
            CUtil.getName(node.getReaction().getParent().tpr) + "." + node.getReaction().getName();
        // Need a positive number to name the nodes in GraphViz
        var labelHashCode = label.hashCode() & 0xfffffff;
        dotRepresentation.pr("    node_" + labelHashCode + " [label=\"" + label + "\"];");

        // Draw the edges
        var downstreamNodes = getDownstreamAdjacentNodes(node);
        for (var downstreamNode : downstreamNodes) {
          var downstreamLabel =
              CUtil.getName(downstreamNode.getReaction().getParent().tpr)
                  + "."
                  + downstreamNode.getReaction().getName();
          edges
              .append("    node_")
              .append(labelHashCode)
              .append(" -> node_")
              .append(downstreamLabel.hashCode() & 0xfffffff)
              .append(";\n");
        }
      }
      // Close the subgraph
      dotRepresentation.pr("}");
    }
    dotRepresentation.unindent();
    // Add the edges to the definition of the graph at the bottom
    dotRepresentation.pr(edges);
    // Close the digraph
    dotRepresentation.pr("}");

    // Return the DOT representation
    return dotRepresentation.toString();
  }
}
