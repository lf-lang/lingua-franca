package org.lflang.generator;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * A position in a document, including line and column. This position may be relative to some
 * position other than the origin.
 *
 * @author Peter Donovan
 */
public class Position implements Comparable<Position> {
  public static final Pattern PATTERN = Pattern.compile("\\((?<line>[0-9]+), (?<column>[0-9]+)\\)");

  public static final Position ORIGIN = Position.fromZeroBased(0, 0);

  private static final Pattern LINE_SEPARATOR = Pattern.compile("(\n)|(\r)|(\r\n)");

  private final int line;
  private final int column;

  /* ------------------------  CONSTRUCTORS  -------------------------- */

  /**
   * Return the Position that describes the given zero-based line and column numbers.
   *
   * @param line the zero-based line number
   * @param column the zero-based column number
   * @return a Position describing the position described by {@code line} and {@code column}.
   */
  public static Position fromZeroBased(int line, int column) {
    return new Position(line, column);
  }

  /**
   * Return the Position that describes the given one-based line and column numbers.
   *
   * @param line the one-based line number
   * @param column the one-based column number
   * @return a Position describing the position described by {@code line} and {@code column}.
   */
  public static Position fromOneBased(int line, int column) {
    return new Position(line - 1, column - 1);
  }

  /**
   * Return the Position that equals the displacement caused by {@code text}, assuming that {@code
   * text} starts in column 0.
   *
   * @param text an arbitrary string
   * @return the Position that equals the displacement caused by {@code text}
   */
  public static Position displacementOf(String text) {
    String[] lines = text.lines().toArray(String[]::new);
    if (lines.length == 0) return ORIGIN;
    return Position.fromZeroBased(lines.length - 1, lines[lines.length - 1].length());
  }

  /**
   * Return the Position that describes the same location in {@code content} as {@code offset}.
   *
   * @param offset a location, expressed as an offset from the beginning of {@code content}
   * @param content the content of a document
   * @return the Position that describes the same location in {@code content} as {@code offset}
   */
  public static Position fromOffset(int offset, String content) {
    int lineNumber = 0;
    Matcher matcher = LINE_SEPARATOR.matcher(content);
    int start = 0;
    while (matcher.find(start)) {
      if (matcher.start() > offset) return Position.fromZeroBased(lineNumber, offset - start);
      start = matcher.end();
      lineNumber++;
    }
    return Position.fromZeroBased(lineNumber, offset);
  }

  /**
   * Create a new Position with the given line and column numbers.
   *
   * @param line the zero-based line number
   * @param column the zero-based column number
   */
  private Position(int line, int column) {
    // Assertions about whether line and column are
    // non-negative are deliberately omitted. Positions
    // can be relative.
    this.line = line;
    this.column = column;
  }

  /* -----------------------  PUBLIC METHODS  ------------------------- */

  /**
   * Return the one-based line number described by this {@code Position}.
   *
   * @return the one-based line number described by this {@code Position}
   */
  public int getOneBasedLine() {
    return line + 1;
  }

  /**
   * Return the one-based column number described by this {@code Position}.
   *
   * @return the one-based column number described by this {@code Position}
   */
  public int getOneBasedColumn() {
    return column + 1;
  }

  /**
   * Return the zero-based line number described by this {@code Position}.
   *
   * @return the zero-based line number described by this {@code Position}
   */
  public int getZeroBasedLine() {
    return line;
  }

  /**
   * Return the zero-based column number described by this {@code Position}.
   *
   * @return the zero-based column number described by this {@code Position}
   */
  public int getZeroBasedColumn() {
    return column;
  }

  /**
   * Return the Position that equals the displacement of ((text whose displacement equals {@code
   * this}) concatenated with {@code text}). Note that this is not necessarily equal to ({@code
   * this} + displacementOf(text)).
   *
   * @param text an arbitrary string
   * @return the Position that equals the displacement caused by {@code text}
   */
  public Position plus(String text) {
    text += "\n"; // Turn line separators into line terminators.
    String[] lines = text.lines().toArray(String[]::new);
    if (lines.length == 0) return this; // OK not to copy because Positions are immutable
    int lastLineLength = lines[lines.length - 1].length();
    return new Position(
        line + lines.length - 1, lines.length > 1 ? lastLineLength : column + lastLineLength);
  }

  /**
   * Return the sum of this and another {@code Position}. The result has meaning because Positions
   * are relative.
   *
   * @param other another {@code Position}
   * @return the sum of this and {@code other}
   */
  public Position plus(Position other) {
    return new Position(line + other.line, column + other.column);
  }

  /**
   * Return the difference of this and another {@code Position}. The result has meaning because
   * Positions are relative.
   *
   * @param other another {@code Position}
   * @return the difference of this and {@code other}
   */
  public Position minus(Position other) {
    return new Position(line - other.line, column - other.column);
  }

  /**
   * Compare two positions according to their order of appearance in a document (first according to
   * line, then according to column).
   */
  @Override
  public int compareTo(Position o) {
    if (line != o.line) {
      return line - o.line;
    }
    return column - o.column;
  }

  @Override
  public boolean equals(Object obj) {
    return obj instanceof Position && ((Position) obj).compareTo(this) == 0;
  }

  @Override
  public String toString() {
    return String.format("(%d, %d)", getZeroBasedLine(), getZeroBasedColumn());
  }

  /**
   * Return the Position represented by {@code s}.
   *
   * @param s a String that represents a Position, formatted like the output of {@code
   *     Position::toString}.
   * @return the Position represented by {@code s}
   */
  public static Position fromString(String s) {
    Matcher matcher = PATTERN.matcher(s);
    if (matcher.matches()) {
      return Position.fromZeroBased(
          Integer.parseInt(matcher.group("line")), Integer.parseInt(matcher.group("column")));
    }
    throw new IllegalArgumentException(String.format("Could not parse %s as a Position.", s));
  }

  @Override
  public int hashCode() {
    return line * 31 + column;
  }

  /**
   * Remove the names from the named capturing groups that appear in {@code regex}.
   *
   * @param regex an arbitrary regular expression
   * @return a string representation of {@code regex} with the names removed from the named
   *     capturing groups
   */
  public static String removeNamedCapturingGroups(Pattern regex) { // FIXME: Does this belong here?
    return regex.toString().replaceAll("\\(\\?<\\w+>", "(");
  }
}
