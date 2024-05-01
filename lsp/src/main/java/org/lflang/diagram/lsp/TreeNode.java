package org.lflang.diagram.lsp;

import org.eclipse.emf.common.util.URI;

public class TreeNode {
    private String label;
    private String uri;
    private NodePosition position;

    public TreeNode(String label, String uri, NodePosition position) {
        this.label = label;
        this.uri = uri;
        this.position = position;
    }

    public String getLabel() {
        return label;
    }

    public String getUri() {
        return uri;
    }

    public NodePosition getPosition() { return position;}
}
