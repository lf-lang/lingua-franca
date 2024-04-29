package org.lflang.diagram.lsp;

import org.eclipse.emf.common.util.URI;
import java.util.ArrayList;
import java.util.List;

public class Tree {
    private String label;
    private String uri;
    private List<TreeNode> children;

    public Tree(String uri) {
        this.uri = uri;
        String[] splits = uri.toString().split("/");
        this.label = splits[splits.length - 1];
        this.children = new ArrayList<>();
    }

    public String getLabel() {
        return label;
    }

    public String getUri() {
        return uri;
    }

    public void addChild(TreeNode child) {
        children.add(child);
    }

    public List<TreeNode> getChildren() {
        return children;
    }
}
