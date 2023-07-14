package org.lflang.analyses.dag;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.StringTokenizer;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.eclipse.xtext.xbase.lib.Exceptions;
import org.lflang.TimeValue;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactionInstance;

/**
 * Class representing a Directed Acyclic Graph (Dag) as an array of Dag edges and an array of Dag
 * nodes. The Dag is then used to generate the dependency matrix, useful for the static scheduling.
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

  /** Head of the Dag */
  public DagNode head;

  /** Tail of the Dag */
  public DagNode tail;

  /**
   * An array of partitions, where each partition is a set of nodes. The index of the partition is
   * the worker ID that owns the partition.
   */
  public List<List<DagNode>> partitions = new ArrayList<>();

  /** A dot file that represents the diagram */
  private CodeBuilder dot;

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
    
    // copy the head and tail nodes
    this.head = other.head;
    this.tail = other.tail;    
  }

  /**
   * Deep copies a HashMap<DagNode, HashMap<DagNode, DagEdge>>.
   * This is necessary because we want the copied Dag to have completely separate collections,
   * and not just separate outer HashMaps that contain references to the same inner HashMaps.
   *
   * @param original the HashMap to be copied
   * @return a deep copy of the original HashMap
   */
  private static HashMap<DagNode, HashMap<DagNode, DagEdge>> deepCopyHashMap(HashMap<DagNode, HashMap<DagNode, DagEdge>> original) {
    HashMap<DagNode, HashMap<DagNode, DagEdge>> copy = new HashMap<>();
    for (DagNode key : original.keySet()) {
      copy.put(key, new HashMap<>(original.get(key)));
    }
    return copy;
  }

  /**
   * Add a SYNC or DUMMY node
   *
   * @param type should be either DYMMY or SYNC
   * @param timeStep either the time step or the time
   * @return the construted Dag node
   */
  public DagNode addNode(DagNode.dagNodeType type, TimeValue timeStep) {
    DagNode dagNode = new DagNode(type, timeStep);
    this.dagNodes.add(dagNode);
    return dagNode;
  }

  /**
   * Add a REACTION node
   *
   * @param type should be REACTION
   * @param reactionInstance
   * @return the construted Dag node
   */
  public DagNode addNode(DagNode.dagNodeType type, ReactionInstance reactionInstance) {
    DagNode dagNode = new DagNode(type, reactionInstance);
    this.dagNodes.add(dagNode);
    return dagNode;
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

  /**
   * Generate a dot file from the DAG.
   *
   * @return a CodeBuilder with the generated code
   */
  public CodeBuilder generateDot() {
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
      if (node.nodeType == DagNode.dagNodeType.SYNC) {
        label = "label=\"Sync" + "@" + node.timeStep;
        auxiliaryNodes.add(i);
      } else if (node.nodeType == DagNode.dagNodeType.DUMMY) {
        label = "label=\"Dummy" + "=" + node.timeStep;
        auxiliaryNodes.add(i);
      } else if (node.nodeType == DagNode.dagNodeType.REACTION) {
        label =
            "label=\""
                + node.nodeReaction.getFullName()
                + "\n"
                + "WCET="
                + node.nodeReaction.wcet
                + (node.getWorker() >= 0 ? "\nWorker=" + node.getWorker() : "");
      } else {
        // Raise exception.
        System.out.println("UNREACHABLE");
        System.exit(1);
      }

      // Add debug message, if any.
      label += node.getDotDebugMsg().equals("") ? "" : "\n" + node.getDotDebugMsg();
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

  public void generateDotFile(Path filepath) {
    try {
      CodeBuilder dot = generateDot();
      String filename = filepath.toString();
      dot.writeToFile(filename);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  /**
   * Parses the dot file, reads the edges and updates the DAG. We assume that the edges are
   * specified as: <SrcNodeId> -> <SinkNodeId>.
   *
   * <p>Furthermore, we assume that the new DAG (contained in the dotFile) will not have unnecessary
   * edges, since they are removed by the shceduler.
   *
   * @param dotFilename
   * @return
   */
  public boolean updateDag(String dotFileName) throws IOException {
    FileReader fileReader;
    BufferedReader bufferedReader;
    // Read the file
    try {
      fileReader = new FileReader(dotFileName);
      // Buffer the input stream from the file
      bufferedReader = new BufferedReader(fileReader);
    } catch (IOException e) {
      System.out.println("Problem accessing file " + dotFileName + "! " + e);
      return false;
    }

    String line;

    // Pattern with which an edge starts:
    Pattern pattern = Pattern.compile("^((\s*)(\\d+)(\s*)->(\s*)(\\d+))");
    Matcher matcher;

    // Before iterating to search for the edges, we clear the DAG edges array list
    this.dagEdges.clear();

    // Search
    int i = 0;
    while (bufferedReader.ready()) {

      line = bufferedReader.readLine();
      matcher = pattern.matcher(line);
      if (matcher.find()) {
        // This line describes an edge
        // Start by removing all white spaces. Only the nodes ids and the
        // arrow remain in the string.
        line = line.replaceAll("\\s", "");

        // Remove the label and the ';' that may appear after the edge specification
        StringTokenizer st = new StringTokenizer(line, ";");
        line = st.nextToken();
        st = new StringTokenizer(line, "[");
        line = st.nextToken();

        // Use a StringTokenizer to find the source and sink nodes' ids
        st = new StringTokenizer(line, "->");
        int srcNodeId, sinkNodeId;

        // Get the source and sink nodes ids and add the edge
        try {
          srcNodeId = Integer.parseInt(st.nextToken());
          sinkNodeId = Integer.parseInt(st.nextToken());
          this.addEdge(srcNodeId, sinkNodeId);
        } catch (NumberFormatException e) {
          System.out.println("Parse error in line " + line + " : Expected a number!");
          Exceptions.sneakyThrow(e);
        }
      }
    }
    bufferedReader.close();
    return true;
  }
}
