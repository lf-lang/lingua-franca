package org.lflang.diagram.lsp;

import java.util.ArrayList;
import java.util.List;

/** Represents a tree structure, where each node can have zero or more children nodes. */
public class LibraryFileTree {
  private String label; // The label associated with the tree.
  private String uri; // The URI specifying the location of the tree.
  private List<LibraryFileTreeNode> children; // The list of children nodes of the tree.

  /**
   * Constructs a new Tree with the specified URI.
   *
   * @param uri The URI specifying the location of the tree.
   */
  public LibraryFileTree(String uri) {
    this.uri = uri;
    String[] splits = uri.toString().split("/");
    this.label = splits[splits.length - 1];
    this.children = new ArrayList<>();
  }

  /**
   * Retrieves the label of the tree.
   *
   * @return The label of the tree.
   */
  public String getLabel() {
    return label;
  }

  /**
   * Retrieves the URI specifying the location of the tree.
   *
   * @return The URI specifying the location of the tree.
   */
  public String getUri() {
    return uri;
  }

  /**
   * Adds a child node to the tree.
   *
   * @param child The child node to be added.
   */
  public void addChild(LibraryFileTreeNode child) {
    children.add(child);
  }

  /**
   * Retrieves the list of children nodes of the tree.
   *
   * @return The list of children nodes of the tree.
   */
  public List<LibraryFileTreeNode> getChildren() {
    return children;
  }
}
