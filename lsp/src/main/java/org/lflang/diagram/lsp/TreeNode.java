package org.lflang.diagram.lsp;

import org.eclipse.emf.common.util.URI;

public class TreeNode {
    private String label;
    private String uri;

    public TreeNode(String label, String uri) {
        this.label = label;
        this.uri = uri;
    }

    public String getLabel() {
        return label;
    }

    public String getUri() {
        return uri;
    }
}
