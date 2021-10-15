package org.lflang.generator;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.jetbrains.annotations.NotNull;

/**
 * <p>
 *     Represents a range in a document. Ranges have a
 *     natural ordering that respects their start
 *     offset(s) only.
 * </p>
 * <p>
 *     Note: This class has a natural ordering that is
 *     inconsistent with <code>equals</code>.
 * </p>
 */
public class Range implements Comparable<Range> {
    public static final Pattern PATTERN = Pattern.compile("Range: \\[(?<start>[0-9]+), (?<end>[0-9]+)\\)");

    /** The start of the Range (INCLUSIVE). */
    private final int start;
    /** The end of the Range (EXCLUSIVE). */
    private final int end;

    /* ------------------------- PUBLIC METHODS -------------------------- */

    /**
     * Initializes a Range that starts at
     * <code>startInclusive</code> and ends at, but DOES NOT
     * INCLUDE, <code>endExclusive</code>.
     * @param startInclusive the start of the range
     *                       (inclusive)
     * @param endExclusive the end of the range (exclusive)
     */
    public Range(int startInclusive, int endExclusive) {
        assert startInclusive <= endExclusive: "endExclusive cannot precede startInclusive";
        start = startInclusive;
        end = endExclusive;
    }

    public int getStartInclusive() {
        return start;
    }

    public int getEndExclusive() {
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
        return this.start - o.start;
    }

    /**
     * Returns whether this contains <code>p</code>.
     * @param p an arbitrary offset
     * @return whether this contains <code>p</code>
     */
    public boolean contains(int p) {
        return start <= p && p < end;
    }

    @Override
    public String toString() {
        return String.format("Range: [%s, %s)", start, end);
    }

    public static Range fromString(String s) {
        return fromString(s, 0);
    }

    public static Range fromString(String s, int relativeTo) {
        Matcher matcher = PATTERN.matcher(s);
        if (matcher.matches()) {
            int start = Integer.parseInt(matcher.group("start"));
            int end = Integer.parseInt(matcher.group("end"));
            return new Range(relativeTo + start, relativeTo + end);
        }
        throw new IllegalArgumentException(String.format("Could not parse %s as a Range.", s));
    }

    /**
     * Returns the degenerate range that simply
     * describes the exact location specified by <code>p
     * </code>.
     * @param p an arbitrary offset
     * @return a Range that starts and ends immediately
     * before <code>p</code>
     */
    public static Range degenerateRange(int p) {
        return new Range(p, p);
    }
}
