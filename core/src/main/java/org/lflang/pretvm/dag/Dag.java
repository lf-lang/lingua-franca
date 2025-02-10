package org.lflang.pretvm.dag;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Queue;
import java.util.Set;
import java.util.StringTokenizer;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.stream.Collectors;
import org.lflang.TimeValue;
import org.lflang.pretvm.instruction.Instruction;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactionInstance;

/**
 * Class representing a Directed Acyclic Graph (Dag), useful for the static scheduling.
 *
 * 
 * 
 */
public class Dag {

  /**
   * Array of Dag nodes. It has to be an array, not a set, because nodes can be duplicated at
   * different positions. Also because the order helps with the dependency generation.
   */
  public ArrayList<DagNode> dagNodes = new ArrayList<DagNode>();

  /** Array of directed edges. Look up an edge using dagEdges.get(source).get(sink). */
  public HashMap<DagNode, HashMap<DagNode, DagEdge>> dagEdges = new HashMap<>();

  /**
   * Array of directed edges in a reverse direction. Look up an edge using
   * dagEdges.get(sink).get(source).
   */
  public HashMap<DagNode, HashMap<DagNode, DagEdge>> dagEdgesRev = new HashMap<>();

  /** Start of the Dag task set, also the head of the Dag */
  public DagNode start;

  /**
   * End of the Dag task set. This end comes from a local hyperperiod, but here we also interpret it
   * as a physical deadline, because we want to start the next iteration of the Dag (i.e.,
   * hyperperiod) with zero lag.
   */
  public DagNode end;

  /**
   * Tail of the Dag. This might not be the end of the Dag task set because of the SYNC node
   * generated by deadlines. We only store this because this is the actual tail node of the graph,
   * though it might not contribute any code in code generation.
   */
  public DagNode tail;

  /**
   * An array of partitions, where each partition is a set of nodes. The index of the partition is
   * the worker ID that owns the partition.
   */
  public List<List<DagNode>> partitions = new ArrayList<>();

  /**
   * A list of worker names that identify specific workers (e.g., core A on board B), with the order
   * matching that of partitions
   */
  public List<String> workerNames = new ArrayList<>();

  /**
   * Store the dependencies between a downstream node (the map key) and its upstream nodes (the map
   * value). The downstream node needs to wait until all of its upstream nodes complete. The static
   * scheduler might prune away some information from the raw DAG (e.g., redundant edges). This map
   * is used to remember some dependencies that we do not want to forget after the static scheduler
   * does its work. These dependencies are later used during instruction generation.
   */
  public Map<DagNode, List<DagNode>> waitUntilDependencies = new HashMap<>();

  /** A dot file that represents the diagram */
  private CodeBuilder dot;

  /** A cache of the same Dag nodes sorted in topological order. */
  private List<DagNode> cachedTopologicalSort;

  /** Constructor */
  public Dag() {}

  /**
   * Copy constructor.
   *
   * @param other the Dag object to be copied
   */
  public Dag(Dag other) {
    // create new collections with the contents of the other Dag
    this.dagNodes = new ArrayList<>(other.dagNodes);
    this.dagEdges = deepCopyHashMap(other.dagEdges);
    this.dagEdgesRev = deepCopyHashMap(other.dagEdgesRev);
    this.partitions = new ArrayList<>();
    for (List<DagNode> partition : other.partitions) {
      this.partitions.add(new ArrayList<>(partition));
    }

    // Copy special nodes.
    this.start = other.start;
    this.end = other.end;
    this.tail = other.tail;
  }

  /**
   * Deep copies a HashMap<DagNode, HashMap<DagNode, DagEdge>>. This is necessary because we want
   * the copied Dag to have completely separate collections, and not just separate outer HashMaps
   * that contain references to the same inner HashMaps.
   *
   * @param original the HashMap to be copied
   * @return a deep copy of the original HashMap
   */
  private static HashMap<DagNode, HashMap<DagNode, DagEdge>> deepCopyHashMap(
      HashMap<DagNode, HashMap<DagNode, DagEdge>> original) {
    HashMap<DagNode, HashMap<DagNode, DagEdge>> copy = new HashMap<>();
    for (DagNode key : original.keySet()) {
      copy.put(key, new HashMap<>(original.get(key)));
    }
    return copy;
  }

  /**
   * Add a node to the DAG.
   *
   * @param node the node to be added
   */
  public void addNode(DagNode node) {
    // If the node is a job node for a reaction,
    // add the number of invocations the reaction has fired so far to the node.
    if (node instanceof JobNode job) {
      job.setReactionRepeatCount(dagNodes.stream().filter(it -> it.isSynonyous(node)).toList().size());
    }
    this.dagNodes.add(node);
  }

  /**
   * Add an edge to the Dag, where the parameters are two DagNodes.
   *
   * @param source
   * @param sink
   */
  public void addEdge(DagNode source, DagNode sink) {

    DagEdge dagEdge = new DagEdge(source, sink);

    if (this.dagEdges.get(source) == null)
      this.dagEdges.put(source, new HashMap<DagNode, DagEdge>());
    if (this.dagEdgesRev.get(sink) == null)
      this.dagEdgesRev.put(sink, new HashMap<DagNode, DagEdge>());

    if (this.dagEdges.get(source).get(sink) == null) this.dagEdges.get(source).put(sink, dagEdge);
    if (this.dagEdgesRev.get(sink).get(source) == null)
      this.dagEdgesRev.get(sink).put(source, dagEdge);
  }

  /**
   * Add an edge to the Dag, where the parameters are the indexes of two DagNodes in the dagNodes
   * array.
   *
   * @param srcNodeId index of the source DagNode
   * @param sinkNodeId index of the sink DagNode
   * @return true, if the indexes exist and the edge is added, false otherwise.
   */
  public boolean addEdge(int srcNodeId, int sinkNodeId) {
    if (srcNodeId < this.dagNodes.size() && sinkNodeId < this.dagNodes.size()) {
      // Get the DagNodes
      DagNode srcNode = this.dagNodes.get(srcNodeId);
      DagNode sinkNode = this.dagNodes.get(sinkNodeId);
      // Add the edge
      this.addEdge(srcNode, sinkNode);
      return true;
    }
    return false;
  }

  /**
   * Remove an edge to the Dag, where the parameters are two DagNodes.
   *
   * @param source
   * @param sink
   */
  public void removeEdge(DagNode source, DagNode sink) {
    if (this.dagEdges.get(source) != null) this.dagEdges.get(source).remove(sink);
    if (this.dagEdgesRev.get(sink) != null) this.dagEdgesRev.get(sink).remove(source);
  }

  /** Clear all the edges of this DAG. */
  public void clearAllEdges() {
    this.dagEdges.clear();
    this.dagEdgesRev.clear();
  }

  /** Add a dependency for WU generation, if two nodes are mapped to different workers. */
  public void addWUDependency(DagNode downstream, DagNode upstream) {
    if (waitUntilDependencies.get(downstream) == null) {
      waitUntilDependencies.put(downstream, new ArrayList<>(Arrays.asList(upstream)));
    } else {
      waitUntilDependencies.get(downstream).add(upstream);
    }
  }

  /**
   * Check if the Dag edge and node lists are empty.
   *
   * @return true, if edge and node arrays are empty, false otherwise
   */
  public boolean isEmpty() {
    if (this.dagEdges.size() == 0 && this.dagNodes.size() == 0) return true;
    return false;
  }

  /**
   * Check if the edge already exits, based on the nodes indexes
   *
   * @param srcNodeId index of the source DagNode
   * @param sinkNodeId index of the sink DagNode
   * @return true, if the edge is already in dagEdges array, false otherwise.
   */
  public boolean edgeExists(int srcNodeId, int sinkNodeId) {
    if (srcNodeId < this.dagEdges.size() && sinkNodeId < this.dagEdges.size()) {
      // Get the DagNodes.
      DagNode srcNode = this.dagNodes.get(srcNodeId);
      DagNode sinkNode = this.dagNodes.get(sinkNodeId);
      HashMap<DagNode, DagEdge> map = this.dagEdges.get(srcNode);
      if (map == null) return false;
      DagEdge edge = map.get(sinkNode);
      if (edge != null) return true;
    }
    return false;
  }

  /** Return an array list of DagEdge */
  public List<DagEdge> getDagEdges() {
    return dagEdges.values().stream()
        .flatMap(innerMap -> innerMap.values().stream())
        .collect(Collectors.toCollection(ArrayList::new));
  }

  /**
   * Get the immediate downstream nodes of a given node.
   *
   * @param node the node to get the downstream nodes of
   * @return a list of downstream nodes
   */
  public List<DagNode> getDownstreamNodes(DagNode node) {
    return new ArrayList<>(this.dagEdges.getOrDefault(node, new HashMap<>()).keySet());
  }

  /**
   * Get the immediate upstream nodes of a given node.
   *
   * @param node the node to get the upstream nodes of
   * @return a list of upstream nodes
   */
  public List<DagNode> getUpstreamNodes(DagNode node) {
    return new ArrayList<>(this.dagEdgesRev.getOrDefault(node, new HashMap<>()).keySet());
  }

  /**
   * Sort the dag nodes by the topological order, i.e., if node B depends on node A, then A has a
   * smaller index than B in the list.
   *
   * @return A topologically sorted list of dag nodes
   */
  public List<DagNode> getTopologicalSort() {
    if (cachedTopologicalSort != null) return cachedTopologicalSort;

    cachedTopologicalSort = new ArrayList<>();

    // Initialize a queue and a map to hold the indegree of each node.
    Queue<DagNode> queue = new LinkedList<>();
    Map<DagNode, Integer> indegree = new HashMap<>();

    // Initialize indegree of all nodes to be the size of their respective upstream node set.
    for (DagNode node : this.dagNodes) {
      indegree.put(node, this.dagEdgesRev.getOrDefault(node, new HashMap<>()).size());
      // Add the node with zero indegree to the queue.
      if (this.dagEdgesRev.getOrDefault(node, new HashMap<>()).size() == 0) {
        queue.add(node);
      }
    }

    // The main loop for traversal using an iterative topological sort.
    while (!queue.isEmpty()) {
      // Dequeue a node.
      DagNode current = queue.poll();

      // Add the node to the sorted list.
      cachedTopologicalSort.add(current);

      // Visit each downstream node.
      HashMap<DagNode, DagEdge> innerMap = this.dagEdges.get(current);
      if (innerMap != null) {
        for (DagNode n : innerMap.keySet()) {
          // Decrease the indegree of the downstream node.
          int updatedIndegree = indegree.get(n) - 1;
          indegree.put(n, updatedIndegree);

          // If the downstream node has zero indegree now, add it to the queue.
          if (updatedIndegree == 0) {
            queue.add(n);
          }
        }
      }
    }
    return cachedTopologicalSort;
  }

  /**
   * Generate a dot file from the DAG.
   *
   * @return a CodeBuilder with the generated code
   */
  public CodeBuilder generateDot(List<List<Instruction>> instructions) {
    dot = new CodeBuilder();
    dot.pr("digraph DAG {");
    dot.indent();

    // Graph settings
    dot.pr("fontname=\"Calibri\";");
    dot.pr("rankdir=TB;");
    dot.pr("node [shape = circle, width = 2.5, height = 2.5, fixedsize = true];");
    dot.pr("ranksep=2.0;  // Increase distance between ranks");
    dot.pr("nodesep=2.0;  // Increase distance between nodes in the same rank");

    // Define nodes.
    ArrayList<Integer> auxiliaryNodes = new ArrayList<>();
    for (int i = 0; i < dagNodes.size(); i++) {
      DagNode node = dagNodes.get(i);
      String code = "";
      String label = "";
      if (node instanceof TimeNode timeNode) {
        label = "label=\"" + "Sync@" + timeNode.getTime() + "\\n" + "WCET=0 nsec";
        auxiliaryNodes.add(i);
      } else if (node instanceof IntervalNode intervalNode) {
        label =
            "label=\""
                + "Dummy="
                + intervalNode.getInterval().toNanoSeconds()
                + "\\n"
                + "WCET="
                + intervalNode.getInterval().toNanoSeconds()
                + " nsec";
        auxiliaryNodes.add(i);
      } else if (node instanceof JobNode jobNode) {
        label =
            "label=\""
                + jobNode.getReaction().getFullName()
                + (jobNode.getWorker() >= 0 ? "\\n" + "Worker=" + jobNode.getWorker() : "");
        label += "\\n" + "WCET=" + jobNode.getReaction().wcet;
      } else {
        // Raise exception.
        throw new RuntimeException("UNREACHABLE");
      }

      // Add PretVM instructions.
      if (instructions != null) {
        if (node instanceof JobNode jobNode) {
          int worker = jobNode.getWorker();
          List<Instruction> workerInstructions = instructions.get(worker);
          if (node.filterInstructions(workerInstructions).size() > 0)
            label += "\\n" + "Instructions:";
          for (Instruction inst : node.filterInstructions(workerInstructions)) {
            label += "\\n" + inst.getOpcode();
          }
          // Add repetition count, if any.
          label += jobNode.getReactionRepeatCount() >= 0 ? "\\n" + "repeat=" + jobNode.getReactionRepeatCount() : "";
        }
        else if (node instanceof TimeNode timeNode) {
          int workers = instructions.size();
          for (int worker = 0; worker < workers; worker++) {
            List<Instruction> workerInstructions = instructions.get(worker);
            for (Instruction inst : node.filterInstructions(workerInstructions)) {
              label += "\\n" + inst.getOpcode() + " (worker " + inst.getWorker() + ")";
            }
          }
        }
      }

      // Add fillcolor and style
      label += "\", fillcolor=\"" + node.getColor() + "\", style=\"filled\"";

      code += i + "[" + label + "]";
      dot.pr(code);
    }

    // Align auxiliary nodes.
    dot.pr("{");
    dot.indent();
    dot.pr("rank = same;");
    for (Integer i : auxiliaryNodes) {
      dot.pr(i + "; ");
    }
    dot.unindent();
    dot.pr("}");

    // Add edges
    for (DagNode source : this.dagEdges.keySet()) {
      HashMap<DagNode, DagEdge> inner = this.dagEdges.get(source);
      if (inner != null) {
        for (DagNode sink : inner.keySet()) {
          int sourceIdx = dagNodes.indexOf(source);
          int sinkIdx = dagNodes.indexOf(sink);
          dot.pr(sourceIdx + " -> " + sinkIdx);
        }
      }
    }

    dot.unindent();
    dot.pr("}");

    return this.dot;
  }

  /**
   * Generate a DOT file without PretVM instructions labeled.
   *
   * @param filepath Filepath to generate the DOT file.
   */
  public void generateDotFile(Path filepath) {
    try {
      CodeBuilder dot = generateDot(null);
      String filename = filepath.toString();
      dot.writeToFile(filename);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  /**
   * Generate a DOT file with PretVM instructions labeled.
   *
   * @param filepath Filepath to generate the DOT file.
   * @param instructions Instructions in a PretVM object file that corresponds to a phase.
   */
  public void generateDotFile(Path filepath, List<List<Instruction>> instructions) {
    try {
      CodeBuilder dot = generateDot(instructions);
      String filename = filepath.toString();
      dot.writeToFile(filename);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  /**
   * Check if the graph is a valid DAG (i.e., it is acyclic).
   *
   * @return true if the graph is a valid DAG, false otherwise.
   */
  public boolean isValidDAG() {
    HashSet<DagNode> whiteSet = new HashSet<>();
    HashSet<DagNode> graySet = new HashSet<>();
    HashSet<DagNode> blackSet = new HashSet<>();

    // Counter for unique IDs
    int[] counter = {0}; // Using an array to allow modification inside the DFS method

    // Initially all nodes are in white set
    whiteSet.addAll(dagNodes);

    while (whiteSet.size() > 0) {
      DagNode current = whiteSet.iterator().next();
      if (dfs(current, whiteSet, graySet, blackSet, counter)) {
        return false;
      }
    }

    return true;
  }

  /**
   * Modified DFS method to assign unique IDs to the nodes.
   *
   * @param current The current node
   * @param whiteSet Set of unvisited nodes
   * @param graySet Set of nodes currently being visited
   * @param blackSet Set of visited nodes
   * @param counter Array containing the next unique ID to be assigned
   * @return true if a cycle is found, false otherwise
   */
  private boolean dfs(
      DagNode current,
      HashSet<DagNode> whiteSet,
      HashSet<DagNode> graySet,
      HashSet<DagNode> blackSet,
      int[] counter) {

    // Move current to gray set
    moveVertex(current, whiteSet, graySet);

    // Visit all neighbors
    HashMap<DagNode, DagEdge> neighbors = dagEdges.get(current);
    if (neighbors != null) {
      for (DagNode neighbor : neighbors.keySet()) {
        // If neighbor is in black set, it means it's already explored, so continue.
        if (blackSet.contains(neighbor)) {
          continue;
        }
        // If neighbor is in gray set then a cycle is found.
        if (graySet.contains(neighbor)) {
          return true;
        }
        if (dfs(neighbor, whiteSet, graySet, blackSet, counter)) {
          return true;
        }
      }
    }

    // Move current to black set and return false
    moveVertex(current, graySet, blackSet);
    return false;
  }

  /**
   * Move a vertex from one set to another.
   *
   * @param vertex The vertex to move
   * @param source The source set
   * @param destination The destination set
   */
  private void moveVertex(DagNode vertex, HashSet<DagNode> source, HashSet<DagNode> destination) {
    source.remove(vertex);
    destination.add(vertex);
  }

  /** Removes redundant edges based on transitive dependencies. */
  public void removeRedundantEdges() {
    List<DagNode> topoSortedNodes = this.getTopologicalSort();
    Set<DagEdge> redundantEdges = new HashSet<>();

    // Map each node to its descendants (transitive closure)
    Map<DagNode, Set<DagNode>> descendants = new HashMap<>();
    for (DagNode node : topoSortedNodes) {
      descendants.put(node, new HashSet<>());
    }

    // Populate the descendants map using the topological sort
    for (DagNode u : topoSortedNodes) {
      Set<DagNode> directDescendants = this.dagEdges.getOrDefault(u, new HashMap<>()).keySet();
      Set<DagNode> allDescendants = descendants.get(u);
      for (DagNode v : directDescendants) {
        allDescendants.add(v);
        allDescendants.addAll(descendants.getOrDefault(v, Collections.emptySet()));
      }
      // Update the descendants of nodes leading to u
      for (DagNode precursor : this.dagEdgesRev.getOrDefault(u, new HashMap<>()).keySet()) {
        descendants.get(precursor).addAll(allDescendants);
      }
    }

    // Identify redundant edges
    for (DagNode u : topoSortedNodes) {
      Set<DagNode> uDescendants = descendants.get(u);
      for (DagNode v : new HashSet<>(uDescendants)) {
        if (this.dagEdges.getOrDefault(u, new HashMap<>()).containsKey(v)) {
          // Check for intermediate nodes
          for (DagNode intermediate : uDescendants) {
            if (this.dagEdges.getOrDefault(intermediate, new HashMap<>()).containsKey(v)) {
              // If such an intermediate exists, the edge u->v is redundant
              redundantEdges.add(this.dagEdges.get(u).get(v));
              break;
            }
          }
        }
      }
    }

    // Remove identified redundant edges
    for (DagEdge edge : redundantEdges) {
      this.removeEdge(edge.sourceNode, edge.sinkNode);
    }
  }
}