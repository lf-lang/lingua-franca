package org.lflang.validation.document;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map.Entry;
import java.util.NavigableMap;

/**
 * Represents a document generated from a Lingua Franca
 * file. Encapsulates any logic associated with the syntax
 * of the document or the extraction of source mappings from
 * it.
 */
abstract class GeneratedDocument {

    private final File directory;
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
     * @param directory the directory in which this
     *                  <code>GeneratedDocument</code> lives
     */
    protected GeneratedDocument(
        List<String> lines,
        NavigableMap<Position, Position> sourceMap,
        File directory
    ) {
        this.directory = directory;
        this.lines = lines;
        this.sourceMap = sourceMap;
    }

    /* -----------------------  PUBLIC METHODS  ------------------------- */

    // FIXME: Account for the possibility of one target document with multiple sources?
    /**
     * Reports diagnostics to <code>acceptor</code> based on
     * the current contents of <code>targetLines</code>.
     * @param acceptor the object that relays diagnostics
     *                 onward to eventually reach the IDE
     */
    public void getDiagnostics(DiagnosticAcceptor acceptor) throws IOException {
        final ProcessBuilder echo = new ProcessBuilder(
            "echo", getCombinedLines()
        );
        final List<ProcessBuilder> pipeline = new ArrayList<>();
        pipeline.add(echo);
        pipeline.add(getVerificationProcess());
        final List<Process> processes = ProcessBuilder.startPipeline(pipeline);
        final BufferedReader reader = new BufferedReader(
            new InputStreamReader(processes.get(1).getErrorStream())
        );
        String line;
        while ((line = reader.readLine()) != null) {
            addDiagnostic(line, acceptor);
        }
    }

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
     * Returns a process that consumes generated document
     * via stdin and emits lines of error messages via
     * stderr.
     * @return a process that consumes generated document
     * via stdin and emits lines of error messages via
     * stderr
     */
    protected abstract ProcessBuilder getVerificationProcess();

    /**
     * Searches line for a diagnostic and, if one is found,
     * adds it to diagnostics.
     * @param line a line of the validator output stream for
     *             a document
     * @param acceptor the object that relays diagnostics
     *                 onward to eventually reach the IDE
     */
    protected abstract void addDiagnostic(String line, DiagnosticAcceptor acceptor);

    /**
     * Returns the directory in which this
     * <code>GeneratedDocument</code> lives.
     * @return the directory in which this
     * <code>GeneratedDocument</code> lives
     */
    protected File getDirectory() {
        return directory;
    }

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
        final int lineDiff = targetPosition.getZeroBasedLine() - nearest.getKey().getZeroBasedLine();
        return Position.fromZeroBased(
            nearest.getValue().getZeroBasedLine() + lineDiff,
            targetPosition.getZeroBasedColumn() // FIXME: Adjust so that it matches the token's location in the source
        );
    }

    /* -----------------------  PRIVATE METHODS  ------------------------ */

}
