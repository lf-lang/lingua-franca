package org.lflang.generator;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.jetbrains.annotations.NotNull;

/**
 * <p>
 *     Represents a range in a document. Ranges have a
 *     natural ordering that respects their start
 *     position(s) only.
 * </p>
 * <p>
 *     Note: This class has a natural ordering that is
 *     inconsistent with <code>equals</code>.
 * </p>
 */
public class Range implements Comparable<Range> {
    public static final Pattern PATTERN = Pattern.compile(String.format(
        "Range: \\[(?<start>%s), (?<end>%s)\\)",
        Position.removeNamedCapturingGroups(Position.PATTERN),
        Position.removeNamedCapturingGroups(Position.PATTERN)
    ));

    /** The start of the Range (INCLUSIVE). */
    private final Position start;
    /** The end of the Range (EXCLUSIVE). */
    private final Position end;

    /* ------------------------- PUBLIC METHODS -------------------------- */

    /**
     * Initializes a Range that starts at
     * <code>startInclusive</code> and ends at, but DOES NOT
     * INCLUDE, <code>endExclusive</code>.
     * @param startInclusive the start of the range
     *                       (inclusive)
     * @param endExclusive the end of the range (exclusive)
     */
    public Range(Position startInclusive, Position endExclusive) {
        assert startInclusive.compareTo(endExclusive) <= 0: "endExclusive cannot precede startInclusive";
        start = startInclusive;
        end = endExclusive;
    }

    public Position getStartInclusive() {
        return start;
    }

    public Position getEndExclusive() {
        return end;
    }

    /**
     * Compares this to <code>o</code>.
     * @param o another Range
     * @return an integer indicating how this compares to
     * <code>o</code>
     */
    @Override
    public int compareTo(@NotNull Range o) {
        return this.start.compareTo(o.start);
    }

    /**
     * Returns whether this contains <code>p</code>.
     * @param p an arbitrary Position
     * @return whether this contains <code>p</code>
     */
    public boolean contains(Position p) {
        return start.compareTo(p) <= 0 && p.compareTo(end) < 0;
    }

    /**
     * Returns the line offset of <code>p</code> from the
     * start of this Range.
     * @param p an arbitrary Position
     * @return the line offset of <code>p</code> from the
     * start of this Range
     */
    public int lineDelta(Position p) {
        return p.getZeroBasedLine() - start.getZeroBasedLine();
    }

    /**
     * Returns the column offset of <code>p</code> from the
     * start of this Range.
     * @param p an arbitrary Position
     * @return the column offset of <code>p</code> from the
     * start of this Range
     */
    public int columnDelta(Position p) {
        return p.getZeroBasedColumn() - start.getZeroBasedColumn();
    }

    @Override
    public String toString() {
        return String.format("Range: [%s, %s)", start, end);
    }

    public static Range fromString(String s) {
        return fromString(s, Position.fromZeroBased(0, 0));
    }

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
     * Returns the degenerate range that simply
     * describes the exact location specified by <code>p
     * </code>.
     * @param p an arbitrary Position
     * @return a Range that starts and ends immediately
     * before <code>p</code>
     */
    public static Range degenerateRange(Position p) {
        return new Range(p, p);
    }
}
