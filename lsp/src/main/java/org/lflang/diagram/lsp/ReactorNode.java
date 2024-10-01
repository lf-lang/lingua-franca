package org.lflang.diagram.lsp;

/**
 * Represents a node in a hierarchical structure where each node has a label, a URI, and positional
 * information. This node typically represents a reactor within a Lingua Franca (LF) file.
 *
 * @see LibraryFile
 */
public class ReactorNode {

  private String label; // The name or label of the reactor.
  private String uri; // The URI indicating the location of the LF file that contains the reactor.
  private NodePosition position; // The position of the reactor within the LF file.

  /**
   * Constructs a new ReactorNode with the specified label, URI, and position.
   *
   * @param label The label or name of the reactor.
   * @param uri The URI that specifies the location of the LF file containing the reactor.
   * @param position The position of the reactor within the LF file.
   */
  public ReactorNode(String label, String uri, NodePosition position) {
    this.label = label;
    this.uri = uri;
    this.position = position;
  }

  /**
   * Returns the label of the reactor.
   *
   * @return The label of the reactor.
   */
  public String getLabel() {
    return label;
  }

  /**
   * Returns the URI of the LF file that contains the reactor.
   *
   * @return The URI of the LF file.
   */
  public String getUri() {
    return uri;
  }

  /**
   * Returns the position information of the reactor within the LF file.
   *
   * @return The position information of the reactor.
   */
  public NodePosition getPosition() {
    return position;
  }
}
