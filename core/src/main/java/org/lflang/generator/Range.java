package org.lflang.generator;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Represents a range in a document. Ranges have a natural ordering that respects their start
 * position(s) only.
 */
public class Range implements Comparable<Range> {
  public static final Pattern PATTERN =
      Pattern.compile(
          String.format(
              "Range: \\[(?<start>%s), (?<end>%s)\\)",
              Position.removeNamedCapturingGroups(Position.PATTERN),
              Position.removeNamedCapturingGroups(Position.PATTERN)));

  /** The start of the Range (INCLUSIVE). */
  private final Position start;

  /** The end of the Range (EXCLUSIVE). */
  private final Position end;

  /* ------------------------- PUBLIC METHODS -------------------------- */

  /**
   * Initializes a Range that starts at {@code startInclusive} and ends at, but does not include,
   * {@code endExclusive}.
   *
   * @param startInclusive the start of the range (inclusive)
   * @param endExclusive the end of the range (exclusive)
   */
  public Range(Position startInclusive, Position endExclusive) {
    assert startInclusive.compareTo(endExclusive) <= 0
        : "endExclusive cannot precede startInclusive";
    start = startInclusive;
    end = endExclusive;
  }

  /**
   * Returns the first Position that is included in this Range.
   *
   * @return the first Position that is included in this Range
   */
  public Position getStartInclusive() {
    return start;
  }

  /**
   * Returns the Position that immediately follows the last Position in this Range.
   *
   * @return the Position that immediately follows the last Position in this Range
   */
  public Position getEndExclusive() {
    return end;
  }

  @Override
  public boolean equals(Object o) {
    if (!(o instanceof Range r)) return false;
    return start.equals(r.start);
  }

  @Override
  public int hashCode() {
    return start.hashCode();
  }

  /**
   * Compares this to {@code o}.
   *
   * @param o another Range
   * @return an integer indicating how this compares to {@code o}
   */
  @Override
  public int compareTo(Range o) {
    return this.start.compareTo(o.start);
  }

  /**
   * Returns whether this contains {@code p}.
   *
   * @param p an arbitrary Position
   * @return whether this contains {@code p}
   */
  public boolean contains(Position p) {
    return start.compareTo(p) <= 0 && p.compareTo(end) < 0;
  }

  @Override
  public String toString() {
    return String.format("Range: [%s, %s)", start, end);
  }

  /**
   * Converts {@code s} to a Range.
   *
   * @param s a String that represents a Range, formatted like the output of {@code Range::toString}
   * @return the Range r such that {@code r.toString()} equals {@code s}
   */
  public static Range fromString(String s) {
    return fromString(s, Position.fromZeroBased(0, 0));
  }

  /**
   * Converts {@code s} to a Range, with the assumption that the positions expressed in {@code s}
   * are given relative to {@code relativeTo}.
   *
   * @param s a String that represents a Range, formatted like the output of {@code Range::toString}
   * @param relativeTo the position relative to which the positions in {@code s} are represented
   * @return the Range represented by {@code s}, expressed relative to the Position relative to
   *     which the Position {@code relativeTo} is expressed
   */
  public static Range fromString(String s, Position relativeTo) {
    Matcher matcher = PATTERN.matcher(s);
    if (matcher.matches()) {
      Position start = Position.fromString(matcher.group("start"));
      Position end = Position.fromString(matcher.group("end"));
      return new Range(start.plus(relativeTo), end.plus(relativeTo));
    }
    throw new IllegalArgumentException(String.format("Could not parse %s as a Range.", s));
  }

  /**
   * Returns the degenerate range that simply describes the exact location specified by {@code p}.
   *
   * @param p an arbitrary Position
   * @return a Range that starts and ends immediately before {@code p}
   */
  public static Range degenerateRange(Position p) {
    return new Range(p, p);
  }
}
