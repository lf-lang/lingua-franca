package org.lflang.validation.document;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

/**
 * Encapsulates logic related to finding document edits
 * at the level of whole lines.
 */
public class DocumentEdit {
    /**
     * Represents an edit to a document at a specific line.
     *
     * All line numbers referenced as part of an edit use
     * zero-based indexing.
     */
    abstract static class Edit implements Iterable<Edit> {
        /**
         * Iterates over a linked list of edits in reverse
         * order.
         */
        static class EditIterator implements Iterator<Edit> {

            private Edit current;

            /**
             * Initializes an <code>EditIterator</code>
             * that starts at <code>current</code> and
             * proceeds backward.
             * @param current the current <code>Edit</code>
             */
            public EditIterator(Edit current) {
                this.current = current;
            }

            @Override
            public boolean hasNext() {
                return current != null;
            }

            @Override
            public Edit next() {
                Edit ret = current;
                current = current.precedingEdit;
                return ret;
            }
        }

        private final int line;
        private final Edit precedingEdit;
        private final int numPrecedingEdits;

        /**
         * Initializes an <code>Edit</code> on the line
         * <code>line</code> whose most recent preceding
         * edit is <code>precedingEdit</code>.
         * @param line the line affected by this
         *             <code>Edit</code> (indexed starting
         *             at zero)
         * @param precedingEdit the most recent
         *                      <code>Edit</code>
         *                      preceding this, or
         *                      <code>null</code> if there
         *                      is no such
         *                      <code>Edit</code>.
         */
        public Edit(int line, Edit precedingEdit) {
            this.line = line;
            this.precedingEdit = precedingEdit;
            this.numPrecedingEdits = precedingEdit == null ? 0 : precedingEdit.numPrecedingEdits + 1;
        }

        @Override
        public Iterator<Edit> iterator() {
            return new EditIterator(this);
        }


        /**
         * Returns the line affected by this
         * <code>Edit</code>.
         * @return the line affected by this
         * <code>Edit</code>
         */
        public int getLine() {
            return line;
        }

        /**
         * Returns the edit corresponding to the minimum
         * cost -- that is, the minimum possible number of
         * edits in the corresponding linked list.
         * @param edits some <code>Edit</code>s that may be
         *              null
         * @return the edit corresponding to the minimum
         * cost -- that is, the minimum possible number of
         * edits in the corresponding linked list
         */
        public static Edit minCost(Edit... edits) {
            Edit ret = edits.length == 0 ? null : edits[0];
            for (int i = 1; i < edits.length; i++) {
                if (numEdits(edits[i]) < numEdits(ret)) {
                    ret = edits[i];
                }
            }
            return ret;
        }

        /**
         * Returns the number of edits in the linked list
         * given by <code>e</code>.
         * @param e an <code>Edit</code>; may be null
         * @return the number of edits in the linked list
         * given by <code>e</code>
         */
        private static int numEdits(Edit e) {
            return e == null ? 0 : e.numPrecedingEdits + 1;
        }
    }

    /**
     * Represents an edit on a specific line that involves
     * the creation of new text.
     */
    abstract static class ConstructiveEdit extends Edit {

        private final String newText;

        /**
         * Initializes a <code>ConstructiveEdit</code> that
         * affects the <code>line</code>th line of a
         * document and involves the creation of the text
         * <code>newText</code>.
         * @param line the line affected by this
         *             <code>ConstructiveEdit</code>
         *             (indexed starting at zero)
         * @param precedingEdit the most recent
         *                      <code>Edit</code>
         *                      preceding this, or
         *                      <code>null</code> if there
         *                      is no such
         *                      <code>Edit</code>.
         * @param newText the newly created text, excluding
         *                any terminating newline
         *                character(s)
         */
        public ConstructiveEdit(int line, Edit precedingEdit, String newText) {
            super(line, precedingEdit);
            this.newText = newText;
        }

        /**
         * Returns the new text constructed in this
         * <code>ConstructiveEdit</code>.
         * @return the new text constructed in this
         * <code>ConstructiveEdit</code>
         */
        public String getNewText() {
            return newText;
        }

        @Override
        public String toString() {
            return this.getClass().getSimpleName() + "(" + getLine() + ", " + newText + ")";
        }
    }

    /**
     * Represents an edit involving the replacement of the
     * text on the <code>line</code>th line with
     * <code>newText</code>.
     */
    static class Mutation extends ConstructiveEdit {
        /**
         * Initializes a <code>ConstructiveEdit</code> that
         * affects the <code>line</code>th line of a
         * document and involves the creation of the text
         * <code>newText</code>.
         *
         * @param line          the line affected by this
         *                      <code>ConstructiveEdit</code>
         *                      (indexed starting at zero)
         * @param precedingEdit the most recent
         *                      <code>Edit</code>
         *                      preceding this, or
         *                      <code>null</code> if there
         *                      is no such
         *                      <code>Edit</code>.
         * @param newText       the newly created text, excluding
         *                      any terminating newline
         */
        public Mutation(int line, Edit precedingEdit, String newText) {
            super(line, precedingEdit, newText);
        }
    }

    /**
     * Represents an insertion of a single new line of text
     * that shifts all lines at or after the line of
     * interest downward by one.
     */
    static class Insertion extends ConstructiveEdit {
        /**
         * Initializes an <code>Insertion</code> of the
         * text <code>newText</code> at the
         * <code>line</code>th line.
         *
         * @param line          the line affected by this
         *                      <code>ConstructiveEdit</code>
         *                      (indexed starting at zero)
         * @param precedingEdit the most recent
         *                      <code>Edit</code>
         *                      preceding this, or
         *                      <code>null</code> if there
         *                      is no such
         *                      <code>Edit</code>.
         * @param newText       the newly created text, excluding
         *                      any terminating newline
         */
        public Insertion(int line, Edit precedingEdit, String newText) {
            super(line, precedingEdit, newText);
        }
    }

    /**
     * Represents the deletion of the <code>line</code>th
     * line.
     */
    static class Deletion extends Edit {

        /**
         * Initializes a <code>Deletion</code> on the line
         * <code>line</code> whose most recent preceding
         * edit is <code>precedingEdit</code>.
         *
         * @param line          the line affected by this
         *                      <code>Edit</code> (indexed
         *                      starting at zero)
         * @param precedingEdit the most recent
         *                      <code>Edit</code>
         *                      preceding this, or
         *                      <code>null</code> if there
         *                      is no such
         */
        public Deletion(int line, Edit precedingEdit) {
            super(line, precedingEdit);
        }

        @Override
        public String toString() {
            return "Deletion(" + getLine() + ")";
        }
    }

    /**
     * Returns the sequence of edits (in decreasing order
     * by line number) required to make
     * <code>editReceiver</code> elementwise-equal to
     * <code>editSource</code>.
     *
     * @implNote Both lists are assumed to permit
     * constant-time random access; without that,
     * time complexity will go from roughly quadratic to
     * roughly cubic in the length of the lists.
     *
     * @param editSource a source of changes
     * @param editReceiver a receiver of changes
     * @return the sequence of edits (in decreasing order
     * by line number) required to make
     * <code>editReceiver</code> elementwise-equal to
     * <code>editSource</code>.
     */
    public static Iterable<Edit> minimalEdit(List<String> editSource, List<String> editReceiver) {
        // The first row corresponds to even values of lengthSum -- visualize this as a diagonal
        //  along a checkerboard, slanting down and to the right. The second row corresponds to
        //  odd values -- the adjacent diagonal.
        Edit[][] fringe = new Edit[2][editReceiver.size() + 1];
        for (int lengthSum = 1; lengthSum <= editSource.size() + editReceiver.size(); lengthSum++) {
            for ( // It is necessary to iterate backwards in order to avoid overwriting needed data
                int receiverLength = Math.min(lengthSum, editReceiver.size());
                receiverLength >= Math.max(0, lengthSum - editSource.size());
                receiverLength--
            ) {
                int sourceLength = lengthSum - receiverLength;
                // It is now necessary to compute the minimal edit required to make the editReceiver sublist of
                // length receiverLength equal to the editSource sublist of length sourceLength.
                Edit result;
                if (sourceLength == 0) {
                    result = new Deletion(receiverLength - 1, fringe[(lengthSum - 1) % 2][receiverLength - 1]);
                } else if (receiverLength == 0) {
                    result = new Insertion(0, fringe[(lengthSum - 1) % 2][receiverLength], editSource.get(0));
                } else if (editSource.get(sourceLength - 1).equals(editReceiver.get(receiverLength - 1))) {
                    result = fringe[(lengthSum - 2) % 2][receiverLength - 1];
                } else {
                    Edit reduceBoth         = fringe[(lengthSum - 2) % 2][receiverLength - 1];
                    Edit reduceEditSource   = fringe[(lengthSum - 1) % 2][receiverLength    ];
                    Edit reduceEditReceiver = fringe[(lengthSum - 1) % 2][receiverLength - 1];
                    Edit chosen = Edit.minCost(reduceBoth, reduceEditReceiver, reduceEditSource);
                    if (chosen == reduceBoth) {
                        result = new Mutation(receiverLength - 1, chosen, editSource.get(sourceLength - 1));
                    } else if (chosen == reduceEditSource) {
                        result = new Insertion(receiverLength, chosen, editSource.get(sourceLength - 1));
                    } else {
                        result = new Deletion(receiverLength - 1, chosen);
                    }
                }
                fringe[lengthSum % 2][receiverLength] = result;
            }
        }
        Edit ret = fringe[(editSource.size() + editReceiver.size()) % 2][editReceiver.size()];
        return ret == null ? new ArrayList<>() : ret;
    }
}
