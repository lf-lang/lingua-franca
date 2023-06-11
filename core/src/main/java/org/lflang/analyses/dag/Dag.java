package org.lflang.analyses.dag;

import org.lflang.TimeValue;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactionInstance;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

/**
 * Class representing a Directed Acyclic Graph (Dag) as an array of Dag edges 
 * and an array of Dag nodes.
 * The Dag is then used to generate the dependency matrix, useful for the static
 * scheduling. 
 */
public class Dag {
    /**
     * Array of Dag nodes. It has to be an array, not a set, because nodes
     * can be duplicated at different positions. Also because the order helps 
     * with the dependency generation.
     */
    public ArrayList<DagNode> dagNodes = new ArrayList<DagNode>();;

    /** 
     * Array of directed edges
     */
    public HashMap<DagNode, HashMap<DagNode, DagEdge>> dagEdges = new HashMap<>();

    /**
     * An array of partitions, where each partition is a set of nodes.
     * The index of the partition is the worker ID that owns the partition.
     */
    public List<List<DagNode>> partitions = new ArrayList<>();

    /**
     * A dot file that represents the diagram
     */
    private CodeBuilder dot;

    /**
     * Add a SYNC or DUMMY node
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
     * @param source
     * @param sink
     */
    public void addEdge(DagNode source, DagNode sink) {
        DagEdge dagEdge = new DagEdge(source, sink);
        if (this.dagEdges.get(source) == null)
            this.dagEdges.put(source, new HashMap<DagNode, DagEdge>());
        this.dagEdges.get(source).put(sink, dagEdge);
    }

    /**
     * Add an edge to the Dag, where the parameters are the indexes of two
     * DagNodes in the dagNodes array.
     * @param srcNodeId index of the source DagNode
     * @param sinkNodeId index of the sink DagNode
     * @return true, if the indexes exist and the edge is added, false otherwise.
     */
    public boolean addEdge(int srcNodeId, int sinkNodeId) {
        if (srcNodeId < this.dagEdges.size()
            && sinkNodeId < this.dagEdges.size()) {
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
     * Check if the Dag edge and node lists are empty.
     * @return true, if edge and node arrays are empty, false otherwise
     */
    public boolean isEmpty() {
        if (this.dagEdges.size() == 0 && this.dagNodes.size() == 0)
            return true;
        return false;
    }

    /**
     * Check if the edge already exits, based on the nodes indexes
     * @param srcNodeId index of the source DagNode
     * @param sinkNodeId index of the sink DagNode
     * @return true, if the edge is already in dagEdges array, false otherwise.
     */
    public boolean edgeExists(int srcNodeId, int sinkNodeId) {
        if (srcNodeId < this.dagEdges.size()
            && sinkNodeId < this.dagEdges.size()) {
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
                label = "label=\"Sync" + "@" + node.timeStep
                    + "\", fillcolor=\"" + node.getColor()
                    + "\", style=\"filled\"";
                auxiliaryNodes.add(i);
            } else if (node.nodeType == DagNode.dagNodeType.DUMMY) {
                label = "label=\"Dummy" + "=" + node.timeStep
                    + "\", fillcolor=\"" + node.getColor() 
                    + "\", style=\"filled\"";
                auxiliaryNodes.add(i);
            } else if (node.nodeType == DagNode.dagNodeType.REACTION) {
                label = "label=\"" + node.nodeReaction.getFullName()
                    + "\nWCET=" + node.nodeReaction.wcet
                    + (node.getWorker() >= 0 ? "\nWorker=" + node.getWorker() : "")
                    + "\", fillcolor=\"" + node.getColor()
                    + "\", style=\"filled\"";
            } else {
                // Raise exception.
                System.out.println("UNREACHABLE");
                System.exit(1);
            }
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
                    int sinkIdx   = dagNodes.indexOf(sink);
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
            e.printStackTrace();
        }
    }
}