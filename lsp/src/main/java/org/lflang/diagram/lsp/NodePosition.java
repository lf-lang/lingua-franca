package org.lflang.diagram.lsp;

/**
 * @brief Position of a node within a text document or code file.
 *     <p>A position is a pair of integers representing the starting and ending line numbers.
 */
public class NodePosition {
  private int start; // The starting position of the node.
  private int end; // The ending position of the node.

  /**
   * Constructs a new NodePosition with the specified start and end positions.
   *
   * @param start The starting position of the node.
   * @param end The ending position of the node.
   */
  public NodePosition(int start, int end) {
    this.start = start;
    this.end = end;
  }

  /**
   * Retrieves the starting position of the node.
   *
   * @return The starting position of the node.
   */
  public int getStart() {
    return start;
  }

  /**
   * Retrieves the ending position of the node.
   *
   * @return The ending position of the node.
   */
  public int getEnd() {
    return end;
  }
}
