package org.lflang.validation.document;

import org.jetbrains.annotations.NotNull;

/**
 * Represents a position in a document, including line and
 * column.
 */
class Position implements Comparable<Position> {
    /*
    Implementation note: This class is designed to remove
    all ambiguity wrt zero-based and one-based line and
    column indexing. The motivating philosophy is that all
    indexing should be zero-based in any programming
    context, unless one is forced to use one-based indexing
    in order to interface with someone else's software. This
    justifies the apparent ambivalence here wrt zero vs.
    one: Zero should be used when possible, but one can be
    used when necessary.

    This philosophy (and the need to be Comparable)
    explains the choice not to use
    org.eclipse.xtext.util.LineAndColumn.
     */

    private final int line;
    private final int column;

    /* ------------------------  CONSTRUCTORS  -------------------------- */

    /**
     * Returns the Position that describes the given
     * zero-based line and column numbers.
     * @param line the zero-based line number
     * @param column the zero-based column number
     * @return a Position describing the position described
     * by <code>line</code> and <code>column</code>.
     */
    public static Position fromZeroBased(int line, int column) {
        return new Position(line, column);
    }

    /**
     * Returns the Position that describes the given
     * one-based line and column numbers.
     * @param line the one-based line number
     * @param column the one-based column number
     * @return a Position describing the position described
     * by <code>line</code> and <code>column</code>.
     */
    public static Position fromOneBased(int line, int column) {
        return new Position(line - 1, column - 1);
    }

    /**
     * Creates a new Position with the given line and column
     * numbers. Private so that unambiguously named factory
     * methods must be used instead.
     * @param line the zero-based line number
     * @param column the zero-based column number
     */
    private Position(int line, int column) {
        this.line = line;
        this.column = column;
    }

    /* -----------------------  PUBLIC METHODS  ------------------------- */

    /**
     * Returns the one-based line number described by this
     * <code>Position</code>.
     * @return the one-based line number described by this
     * <code>Position</code>
     */
    public int getOneBasedLine() {
        return line + 1;
    }

    /**
     * Returns the one-based column number described by this
     * <code>Position</code>.
     * @return the one-based column number described by this
     * <code>Position</code>
     */
    public int getOneBasedColumn() {
        return column + 1;
    }

    /**
     * Returns the zero-based line number described by this
     * <code>Position</code>.
     * @return the zero-based line number described by this
     * <code>Position</code>
     */
    public int getZeroBasedLine() {
        return line;
    }

    /**
     * Returns the zero-based column number described by this
     * <code>Position</code>.
     * @return the zero-based column number described by this
     * <code>Position</code>
     */
    public int getZeroBasedColumn() {
        return column;
    }

    /**
     * Returns a new <code>Position</code> that is
     * <code>lineDelta</code> lines later than this
     * <code>Position</code> and <code>columnDelta</code>
     * columns later than this <code>Position</code>.
     * @param lineDelta the difference in line numbers
     *                  between this position and the
     *                  desired position
     * @param columnDelta the difference in column numbers
     *                    between this position and the
     *                    desired position
     * @return a new <code>Position</code> that is
     * <code>lineDelta</code> lines later than this
     * <code>Position</code> and <code>columnDelta</code>
     * columns later than this <code>Position</code>
     */
    public Position translated(int lineDelta, int columnDelta) {
        return new Position(line + lineDelta, column + columnDelta);
    }

    /**
     * Compares two positions according to their order of
     * appearance in a document (first according to line,
     * then according to column).
     */
    @Override
    public int compareTo(@NotNull Position o) {
        if (line != o.line) {
            return line - o.line;
        }
        return column - o.column;
    }

    @Override
    public boolean equals(Object obj) {
        return obj instanceof Position && ((Position) obj).compareTo(this) == 0;
    }
}
