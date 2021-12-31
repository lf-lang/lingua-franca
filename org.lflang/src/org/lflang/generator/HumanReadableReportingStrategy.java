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
 */
public class HumanReadableReportingStrategy implements DiagnosticReporting.Strategy {

    /** A pattern that matches lines that should be reported via this strategy. */
    private final Pattern diagnosticMessagePattern;
    /** A pattern that matches labels that show the exact range to which the diagnostic pertains. */
    private static final Pattern labelPattern = Pattern.compile("(~*)(\\^~*)");  // TODO: Make this a parameter?

    /**
     * Instantiates a reporting strategy for lines of
     * validator output that match {@code diagnosticMessagePattern}.
     * @param diagnosticMessagePattern a pattern that matches lines that should be
     *          reported via this strategy. This pattern
     *          must contain named capturing groups called
     *          "path", "line", "column", "message", and
     *          "severity".
     */
    public HumanReadableReportingStrategy(Pattern diagnosticMessagePattern) {
        for (String groupName : new String[]{"path", "line", "column", "message", "severity"}) {
            assert diagnosticMessagePattern.pattern().contains(groupName) : String.format(
                "Error line patterns must have a named capturing group called %s", groupName
            );
        }
        this.diagnosticMessagePattern = diagnosticMessagePattern;
    }

    @Override
    public void report(String validationOutput, ErrorReporter errorReporter, Map<Path, CodeMap> map) {
        Iterator<String> it = validationOutput.lines().iterator();
        while (it.hasNext()) reportErrorLine(it.next(), it, errorReporter, map);
    }

    /**
     * Reports the validation message contained in the first line of {@code it},
     * if such a message exists.
     * @param line the current line
     * @param it a line iterator
     * @param errorReporter an arbitrary ErrorReporter
     * @param maps a mapping from generated file paths to
     *             CodeMaps
     */
    private void reportErrorLine(String line, Iterator<String> it, ErrorReporter errorReporter, Map<Path, CodeMap> maps) {
        Matcher matcher = diagnosticMessagePattern.matcher(line);
        if (matcher.matches()) {
            final Path path = Paths.get(matcher.group("path"));
            final Position generatedFilePosition = Position.fromOneBased(
                Integer.parseInt(matcher.group("line")), Integer.parseInt(matcher.group("column"))
            );
            final String message = DiagnosticReporting.messageOf(
                matcher.group("message"), path.getFileName().toString(), generatedFilePosition
            );
            final CodeMap map = maps.get(path);
            final DiagnosticSeverity severity = DiagnosticReporting.severityOf(matcher.group("severity"));
            if (map == null) {
                errorReporter.report(severity, message);
                return;
            }
            for (Path srcFile : map.lfSourcePaths()) {
                // FIXME: Is it desirable for the error to be reported to every single LF file associated
                //  with the generated file containing the error? Or is it best to be more selective?
                Position lfFilePosition = map.adjusted(srcFile, generatedFilePosition);
                reportAppropriateRange(
                    (p0, p1) -> errorReporter.report(severity, message, p0, p1), lfFilePosition, it
                );
            }
        }
    }

    /**
     * Reports the appropriate range to {@code report}.
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
            return;
        }
        reportAppropriateRange(report, lfFilePosition, it);
    }
}
