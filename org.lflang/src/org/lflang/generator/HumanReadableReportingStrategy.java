package org.lflang.generator;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Iterator;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.eclipse.lsp4j.DiagnosticSeverity;
import org.eclipse.xtext.xbase.lib.Procedures.Procedure0;
import org.eclipse.xtext.xbase.lib.Procedures.Procedure2;

import org.lflang.ErrorReporter;

/**
 * An error reporting strategy that parses human-readable
 * output.
 *
 * @author Peter Donovan
 */
public class HumanReadableReportingStrategy implements DiagnosticReporting.Strategy {

    /** A pattern that matches lines that should be reported via this strategy. */
    private final Pattern diagnosticMessagePattern;
    /** A pattern that matches labels that show the exact range to which the diagnostic pertains. */
    private final Pattern labelPattern;
    /** The path against which any paths should be resolved. */
    private final Path relativeTo;
    /** The next line to be processed, or {@code null}. */
    private String bufferedLine;

    /**
     * Instantiate a reporting strategy for lines of
     * validator output that match {@code diagnosticMessagePattern}.
     * @param diagnosticMessagePattern A pattern that matches lines that should be
     *          reported via this strategy. This pattern
     *          must contain named capturing groups called
     *          "path", "line", "column", "message", and
     *          "severity".
     * @param labelPattern A pattern that matches lines that act as labels, showing
     *                     the location of the relevant piece of text. This pattern
     *                     must contain two groups, the first of which must match
     *                     characters that precede the location given by the "line"
     *                     and "column" groups.
     */
    public HumanReadableReportingStrategy(Pattern diagnosticMessagePattern, Pattern labelPattern) {
        this(diagnosticMessagePattern, labelPattern, null);
    }

    /**
     * Instantiate a reporting strategy for lines of
     * validator output that match {@code diagnosticMessagePattern}.
     * @param diagnosticMessagePattern a pattern that matches lines that should be
     *          reported via this strategy. This pattern
     *          must contain named capturing groups called
     *          "path", "line", "column", "message", and
     *          "severity".
     * @param labelPattern A pattern that matches lines that act as labels, showing
     *                     the location of the relevant piece of text. This pattern
     *                     must contain two groups, the first of which must match
     *                     characters that precede the location given by the "line"
     *                     and "column" groups.
     * @param relativeTo The path against which any paths should be resolved.
     */
    public HumanReadableReportingStrategy(Pattern diagnosticMessagePattern, Pattern labelPattern, Path relativeTo) {
        for (String groupName : new String[]{"path", "line", "column", "message", "severity"}) {
            assert diagnosticMessagePattern.pattern().contains(groupName) : String.format(
                "Error line patterns must have a named capturing group called %s", groupName
            );
        }
        this.diagnosticMessagePattern = diagnosticMessagePattern;
        this.labelPattern = labelPattern;
        this.relativeTo = relativeTo;
        this.bufferedLine = null;
    }

    @Override
    public void report(String validationOutput, ErrorReporter errorReporter, Map<Path, CodeMap> map) {
        Iterator<String> it = validationOutput.lines().iterator();
        while (it.hasNext() || bufferedLine != null) {
            if (bufferedLine != null) {
                reportErrorLine(bufferedLine, it, errorReporter, map);
                bufferedLine = null;
            } else {
                reportErrorLine(it.next(), it, errorReporter, map);
            }
        }
    }

    /**
     * Report the validation message contained in the given line of text.
     * @param line The current line.
     * @param it An iterator over the lines that follow the current line.
     * @param errorReporter An arbitrary ErrorReporter.
     * @param maps A mapping from generated file paths to
     *             CodeMaps.
     */
    private void reportErrorLine(String line, Iterator<String> it, ErrorReporter errorReporter, Map<Path, CodeMap> maps) {
        Matcher matcher = diagnosticMessagePattern.matcher(stripEscaped(line));
        if (matcher.matches()) {
            final Path path = Paths.get(matcher.group("path"));
            final Position generatedFilePosition = Position.fromOneBased(
                Integer.parseInt(matcher.group("line")),
                Integer.parseInt(matcher.group("column") != null ? matcher.group("column") : "0") // FIXME: Unreliable heuristic
            );
            final String message = DiagnosticReporting.messageOf(
                matcher.group("message"), path, generatedFilePosition
            );
            final CodeMap map = maps.get(relativeTo != null ? relativeTo.resolve(path) : path);
            final DiagnosticSeverity severity = DiagnosticReporting.severityOf(matcher.group("severity"));
            if (map == null) {
                errorReporter.report(null, severity, message);
                return;
            }
            for (Path srcFile : map.lfSourcePaths()) {
                Position lfFilePosition = map.adjusted(srcFile, generatedFilePosition);
                if (matcher.group("column") != null) {
                    reportAppropriateRange(
                        (p0, p1) -> errorReporter.report(srcFile, severity, message, p0, p1), lfFilePosition, it
                    );
                } else {
                    errorReporter.report(srcFile, severity, message, lfFilePosition.getOneBasedLine());
                }
            }
        }
    }

    /**
     * Report the appropriate range to {@code report}.
     * @param report A reporting method whose first and
     *               second parameters are the (included)
     *               start and (excluded) end of the
     *               relevant range.
     * @param lfFilePosition The point about which the
     *                       relevant range is anchored.
     * @param it An iterator over the lines immediately
     *           following a diagnostic message.
     */
    private void reportAppropriateRange(
        Procedure2<Position, Position> report, Position lfFilePosition, Iterator<String> it
    ) {
        Procedure0 failGracefully = () -> report.apply(lfFilePosition, lfFilePosition.plus(" "));
        if (!it.hasNext()) {
            failGracefully.apply();
            return;
        }
        String line = it.next();
        Matcher labelMatcher = labelPattern.matcher(line);
        if (labelMatcher.find()) {
            report.apply(
                Position.fromZeroBased(
                    lfFilePosition.getZeroBasedLine(),
                    lfFilePosition.getZeroBasedColumn() - labelMatcher.group(1).length()
                ),
                lfFilePosition.plus(labelMatcher.group(2))
            );
            return;
        }
        if (diagnosticMessagePattern.matcher(line).find()) {
            failGracefully.apply();
            bufferedLine = line;
            return;
        }
        reportAppropriateRange(report, lfFilePosition, it);
    }

    /**
     * Strip the ANSI escape sequences from {@code s}.
     * @param s Any string.
     * @return {@code s}, with any escape sequences removed.
     */
    private static String stripEscaped(String s) {
        return s.replaceAll("\u001B\\[[;\\d]*[ -/]*[@-~]", "");
    }
}
