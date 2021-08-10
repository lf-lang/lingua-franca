package org.lflang.ide.document;

import java.io.File;
import java.io.IOException;
import java.util.Collections;
import java.util.List;
import java.util.Map.Entry;
import java.util.NavigableMap;

import org.apache.log4j.Logger;
import org.eclipse.lsp4j.Diagnostic;
import org.eclipse.lsp4j.Position;
import org.eclipse.lsp4j.services.LanguageServer;

/**
 * Represents a document generated from a Lingua Franca
 * file. Encapsulates any logic associated with the syntax
 * of the document or the extraction of source mappings from
 * it.
 */
abstract class GeneratedDocument {

    protected static final Logger LOG = Logger.getLogger(LanguageServer.class);

    private final List<String> lines;
    private final NavigableMap<Position, Position> sourceMap;

    /* ------------------------  CONSTRUCTORS  -------------------------- */

    /**
     * Instantiates a <code>GeneratedDocument</code> with
     * lines of text  and mappings from that text to the
     * source code.
     * @param lines the generated text lines
     * @param sourceMap mappings from positions in the
     *                  generated text to the source code
     */
    protected GeneratedDocument(
        List<String> lines,
        NavigableMap<Position, Position> sourceMap
    ) {
        this.lines = lines;
        this.sourceMap = sourceMap;
    }

    /* -----------------------  PUBLIC METHODS  ------------------------- */

    /**
     * Returns a list of diagnostics based on the current
     * contents of <code>targetLines</code>.
     */
    public abstract List<Diagnostic> getDiagnostics(File workingDir, String extension) throws IOException;

    /**
     * Returns an unmodifiable view of a source map, which
     * includes a (possibly incomplete) set of mappings from
     * positions in the generated document to positions in
     * the source document.
     * @return an unmodifiable view of a source map
     */
    public NavigableMap<Position, Position> getSourceMap() {
        return Collections.unmodifiableNavigableMap(sourceMap);
    }

    /**
     * Updates the content of the <code>line</code>th line.
     * @param line the zero-based line index
     * @param text the new line text (excluding terminating
     *             newline character)
     */
    public void changeLine(int line, String text) {
        // FIXME: Implement.
    }


    /**
     * Removes the <code>line</code>th line.
     * @param line the zero-based line index
     */
    public void deleteLine(int line) {
        // FIXME: Implement.
    }


    /**
     * Inserts a line at the index given by
     * <code>line</code> with the text <code>text</code>.
     * @param line the zero-based line index
     * @param text the new line text (excluding terminating
     *             newline character)
     */
    public void insertLine(int line, String text) {
        // FIXME: Implement.
    }

    /* ---------------------  PROTECTED METHODS  ------------------------ */

    /**
     * Returns an unmodifiable view of the lines in this
     * GeneratedDocument.
     * @return an unmodifiable view of the lines in this
     * GeneratedDocument
     */
    protected List<String> getLines() {
        return Collections.unmodifiableList(lines);
    }

    /**
     * Returns the lines of target code, all concatenated
     * into one String.
     * @return the lines of target code, all concatenated
     * into one String
     */
    protected String getCombinedLines() {
        final StringBuilder builder = new StringBuilder();
        for (String line : getLines()) {
            builder.append(line).append('\n');
        }
        return builder.toString();
    }

    /**
     * Returns the position in the source code corresponding to targetPosition.
     * @param targetPosition a position in the generated target code
     * @return the position in the source code corresponding to targetPosition
     */
    protected Position adjustPosition(Position targetPosition) {
        // TODO add a check to make sure the two lines match up.
        final Entry<Position, Position> nearest = sourceMap.floorEntry(
            targetPosition
        );
        LOG.debug("Nearest- Key: " + nearest.getKey().getLine() + "Value: " + nearest.getValue().getLine());
        LOG.debug("Returned line: " + (nearest.getValue().getLine() + targetPosition.getLine()
            - nearest.getKey().getLine()));
        return new Position(
            nearest.getValue().getLine() + targetPosition.getLine() - nearest.getKey().getLine(),
            targetPosition.getCharacter()
        );
    }

    /* -----------------------  PRIVATE METHODS  ------------------------ */

}
