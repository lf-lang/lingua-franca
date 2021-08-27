package org.lflang.validation.document.generated;

import java.io.BufferedReader;
import java.io.File;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.TreeMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.lflang.validation.document.DiagnosticAcceptor;
import org.lflang.validation.document.LFDocument;
import org.lflang.validation.document.Position;

/**
 * Represents a document generated from a Lingua Franca
 * file. Encapsulates any logic associated with the syntax
 * of the document or the extraction of source mappings from
 * it.
 */
public abstract class GeneratedDocument {
    // FIXME: Account for multiple LF documents that produce code in
    //  the same generated document -- perhaps simply by instantiating
    //  multiple GeneratedDocuments per generated document as needed?

    /** Matches indentation at the beginning of a line. */
    Pattern INDENTATION = Pattern.compile("^ *"); // FIXME: What if '\t' is used?

    /**
     * The directory (typically src-gen) in which this
     * document lives.
     */
    private final File directory;
    /** The content of this <code>GeneratedDocument</code>. */
    private final List<String> lines;
    /**
     * The mappings from positions in this
     * <code>GeneratedDocument</code> to positions in the
     * source LF document.
     */
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
            new InputStreamReader(getVerificationUsesStdout() ? processes.get(1).getInputStream() : processes.get(1).getErrorStream())
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
     * Returns the inverse of the map given by
     * <code>getSourceMap</code>
     * @return the inverse of the map given by
     * <code>getSourceMap</code>
     */
    public NavigableMap<Position, Position> getInverseMap() {
        // FIXME: Everything about this function is wrong. It should be rewritten.
        // FIXME: Cache the result. This is expensive to recompute
        NavigableMap<Position, Position> ret = new TreeMap<>();
        for (Map.Entry<Position, Position> entry : sourceMap.entrySet()) {
            ret.put(entry.getValue(), entry.getKey());
        }
        return ret;
    }

    /**
     * Makes any small adjustments necessary to ensure that
     * the source map tracked by this
     * <code>GeneratedDocument</code> is indeed consistent
     * with the content of <code>source</code>.
     * @param source the source document that this document
     *               model must be consistent with
     */
    public void refineSourceMap(LFDocument source) {
        for (Map.Entry<Position, Position> entry : sourceMap.entrySet()) {
            final String sourceLine = source.getLine(entry.getValue().getZeroBasedLine());
            final String targetLine = lines.get(entry.getKey().getZeroBasedLine());
            final Matcher sourceLineIndentation = INDENTATION.matcher(sourceLine);
            final Matcher targetLineIndentation = INDENTATION.matcher(targetLine);
            int indentationDiff = (sourceLineIndentation.find() ? sourceLineIndentation.group().length() : 0)
                - (targetLineIndentation.find() ? targetLineIndentation.group().length() : 0);
            entry.setValue(entry.getValue().translated(0, indentationDiff));
        }
    }

    /**
     * Updates the content of the <code>line</code>th line.
     * @param line the zero-based line index
     * @param text the new line text (excluding terminating
     *             newline character)
     */
    public void mutateLine(int line, String text) {
        lines.set(inverseAdjustLine(line), text);
        // FIXME: Mappings associated with individual tokens
        //  on this line might become wrong
    }

    /**
     * Inserts a line at the index given by
     * <code>line</code> with the text <code>text</code>.
     * @param line the zero-based line index
     * @param text the new line text (excluding terminating
     *             newline character)
     */
    public void insertLine(int line, String text) {
        line = inverseAdjustLine(line); // FIXME: overwriting a parameter is bad style?
        lines.add(line, text);
        Position ceiling = sourceMap.higherKey(Position.fromZeroBased(line, 0));
        List<Position> affected = List.copyOf(sourceMap.tailMap(ceiling).keySet());
        for (Position p : affected) {
            Position value = sourceMap.get(p);
            sourceMap.remove(p);
            sourceMap.put(p.translated(1, 0), value.translated(1, 0));
        }
    }

    /**
     * Removes the <code>line</code>th line.
     * @param line the zero-based line index
     */
    public void deleteLine(int line) {
        line = inverseAdjustLine(line);
        lines.remove(line);
        Position ceiling = sourceMap.higherKey(Position.fromZeroBased(line, 0));
        List<Position> affected = List.copyOf(sourceMap.tailMap(ceiling).keySet());
        for (Position p : affected) {
            Position value = sourceMap.get(p);
            sourceMap.remove(p);
            if (p.getZeroBasedLine() != line) {
                sourceMap.put(p.translated(-1, 0), value.translated(-1, 0));
            }
        }
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

    protected boolean getVerificationUsesStdout() {
        return false;
    }

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
        return adjustPosition(targetPosition, sourceMap);
    }

    /* -----------------------  PRIVATE METHODS  ------------------------ */

    private int inverseAdjustLine(int line) {
        return inverseAdjustPosition(Position.fromZeroBased(line, 0)).getZeroBasedLine();
    }

    private Position inverseAdjustPosition(Position sourcePosition) {
        return adjustPosition(sourcePosition, getInverseMap());
    }

    private static Position adjustPosition(Position position, NavigableMap<Position, Position> map) {
        // TODO add a check to make sure the two lines match up.
        final Entry<Position, Position> nearest = map.floorEntry(
            position
        );
        final int lineDiff = position.getZeroBasedLine() - nearest.getKey().getZeroBasedLine();
        final int columnDiff = position.getZeroBasedColumn() - nearest.getKey().getZeroBasedColumn();
        return Position.fromZeroBased(
            nearest.getValue().getZeroBasedLine() + lineDiff,
            nearest.getValue().getZeroBasedColumn() + columnDiff // FIXME: Adjust so that it matches the token's location in the source
        );
    }

}
