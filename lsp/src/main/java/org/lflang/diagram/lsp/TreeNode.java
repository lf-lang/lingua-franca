package org.lflang.diagram.lsp;

/**
 * Represents a node in a tree structure, typically used to store information about elements in a hierarchical context.
 */
public class TreeNode {
    private String label;           // The label associated with the node.
    private String uri;             // The URI specifying the location of the associated element.
    private NodePosition position;  // The position information of the associated element.

    /**
     * Constructs a new TreeNode with the specified label, URI, and position.
     *
     * @param label    The label of the node.
     * @param uri      The URI specifying the location of the associated element.
     * @param position The position information of the associated element.
     */
    public TreeNode(String label, String uri, NodePosition position) {
        this.label = label;
        this.uri = uri;
        this.position = position;
    }

    /**
     * Retrieves the label of the node.
     *
     * @return The label of the node.
     */
    public String getLabel() {
        return label;
    }

    /**
     * Retrieves the URI specifying the location of the associated element.
     *
     * @return The URI specifying the location of the associated element.
     */
    public String getUri() {
        return uri;
    }

    /**
     * Retrieves the position information of the associated element.
     *
     * @return The position information of the associated element.
     */
    public NodePosition getPosition() {
        return position;
    }

}

