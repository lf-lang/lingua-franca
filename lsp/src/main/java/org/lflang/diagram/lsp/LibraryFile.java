package org.lflang.diagram.lsp;

import java.util.ArrayList;
import java.util.List;

/**
 * Represents a Lingua Franca (LF) file. This file can contain multiple ReactorNode elements, with
 * each node potentially having zero or more of these reactors.
 */
public class LibraryFile {
  private String label; // The label (name) associated with the LF file.
  private String uri; // The URI specifying the location of the LF file.
  private List<ReactorNode> children; // The list of reactors included within the LF file.

  /**
   * Constructs a new LibraryFile with the specified URI.
   *
   * @param uri The URI specifying the location of the LF file.
   */
  public LibraryFile(String uri) {
    this.uri = uri;
    String[] splits = uri.split("/");
    this.label = splits[splits.length - 1];
    this.children = new ArrayList<>();
  }

  /**
   * Retrieves the label (name) of the LF file.
   *
   * @return The label (name) of the LF file.
   */
  public String getLabel() {
    return label;
  }

  /**
   * Retrieves the URI specifying the location of the LF file.
   *
   * @return The URI specifying the location of the LF file.
   */
  public String getUri() {
    return uri;
  }

  /**
   * Retrieves the list of children nodes of the tree.
   *
   * @return The list of children nodes of the tree.
   */
  public List<ReactorNode> getChildren() {
    return children;
  }
}
