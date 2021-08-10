package org.lflang.ide.document;

import java.util.Comparator;

import org.eclipse.lsp4j.Position;

/**
 * Compares two positions according to their order of
 * appearance in a document (first according to line,
 * then according to column).
 */
class PositionComparator implements Comparator<Position> {
    @Override
    public int compare(Position p1, Position p2) {
        if (p1.getLine() != p2.getLine()) {
            return p1.getLine() - p2.getLine();
        }
        return p1.getCharacter() - p2.getCharacter();
    }
}
