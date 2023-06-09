package org.lflang.analyses.dag;

import org.lflang.TimeValue;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactionInstance;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

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
    public ArrayList<DagNode> dagNodes;

    /** 
     * Array of directed edges
     * 
     * FIXME: Use a nested hashmap for faster lookup.
     * E.g., public HashMap<DagNode, HashMap<DagNode, DagEdge>> dagEdges
     */
    public ArrayList<DagEdge> dagEdges;

    /**
     * Indicates whether this DAG has changed, useful for checking
     * whether a new dot file needs to be generated. 
     */
    public boolean changed = false;

    /**
     * An array of partitions, where each partition is a set of nodes.
     */
    public List<Set<DagNode>> partitions = new ArrayList<>();

    /**
     * A dot file that represents the diagram
     */
    private CodeBuilder dot;

    /**
     * Constructor. Simply creates two array lists.
     */ 
    public Dag(){
        this.dagNodes = new ArrayList<DagNode>();
        this.dagEdges = new ArrayList<DagEdge>();
    }

    /**
     * Add a SYNC or DUMMY node
     * @param type should be either DYMMY or SYNC
     * @param timeStep either the time step or the time
     * @return the construted Dag node
     */
    public DagNode addNode(dagNodeType type, TimeValue timeStep) {
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
    public DagNode addNode(dagNodeType type, ReactionInstance reactionInstance) {
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
        this.dagEdges.add(dagEdge);
    }

    /**
     * Add an edge to the Dag, where the parameters are the indexes of two
     * DagNodes in the dagNodes array.
     * @param srcNodeId index of the source DagNode
     * @param sinkNodeId index of the sink DagNode
     * @return true, if the indexes exist and the edge is added, false otherwise.
     */
    public boolean addEdge(int srcNodeId, int sinkNodeId) {
        if (srcNodeId < this.dagEdges.size() && sinkNodeId < this.dagEdges.size()) {
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
     * 
     * FIXME: ID is not a property of a DAG node, which should be added.
     * The iteration is also an O(n) operation. Using a hashmap is more efficient.
     */
    public boolean edgeExists(int srcNodeId, int sinkNodeId) {
        // Get the DagNodes 
        if (srcNodeId < this.dagEdges.size() && sinkNodeId < this.dagEdges.size()) {
            DagNode srcNode = this.dagNodes.get(srcNodeId);
            DagNode sinkNode = this.dagNodes.get(sinkNodeId);
            // Iterate over the dagEdges array
            for (int i = 0; i < this.dagEdges.size(); i++) {
                DagEdge edge = this.dagEdges.get(i);
                if (edge.sourceNode == srcNode && edge.sinkNode == sinkNode) {
                    return true;
                }
            }
        }
        return false;
    }

    /**
     * Generate a dot file from the DAG.
     * 
     * @return a CodeBuilder with the generated code
     */
    public CodeBuilder generateDot() {
        if (dot == null || changed) {
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
                if (node.nodeType == dagNodeType.SYNC) {
                    label = "label=\"Sync" + "@" + node.timeStep
                        + "\", fillcolor=\"" + node.getColor()
                        + "\", style=\"filled\"";
                    auxiliaryNodes.add(i);
                } else if (node.nodeType == dagNodeType.DUMMY) {
                    label = "label=\"Dummy" + "=" + node.timeStep
                        + "\", fillcolor=\"" + node.getColor() 
                        + "\", style=\"filled\"";
                    auxiliaryNodes.add(i);
                } else if (node.nodeType == dagNodeType.REACTION) {
                    label = "label=\"" + node.nodeReaction.getFullName()
                        + "\nWCET=" + node.getWCET()
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
            for (DagEdge e : dagEdges) {
                int sourceIdx = dagNodes.indexOf(e.sourceNode);
                int sinkIdx   = dagNodes.indexOf(e.sinkNode);
                dot.pr(sourceIdx + " -> " + sinkIdx);
            }

            dot.unindent();
            dot.pr("}");
            
            // If changed is true, now it is okay to unset this flag
            // since we have regenerated the dot file.
            if (changed) changed = false;
        }
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