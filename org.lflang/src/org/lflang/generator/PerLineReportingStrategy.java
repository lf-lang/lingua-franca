package org.lflang.generator;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.lflang.ErrorReporter;

/**
 * An error reporting strategy that considers only one line
 * of validator output at a time.
 */
public class PerLineReportingStrategy implements CommandErrorReportingStrategy {

    private final Pattern p;

    /**
     * Instantiates a reporting strategy for lines of
     * validator output that match {@code p}.
     * @param p a pattern that matches lines that should be
     *          reported via this strategy. This pattern
     *          must contain named capturing groups called
     *          "path", "line", "column", "message", and
     *          "severity".
     */
    public PerLineReportingStrategy(Pattern p) {
        for (String groupName : new String[]{"path", "line", "column", "message", "severity"}) {
            assert p.pattern().contains(groupName) : String.format(
                "Error line patterns must have a named capturing group called %s", groupName
            );
        }
        this.p = p;
    }

    @Override
    public void report(String validationOutput, ErrorReporter errorReporter, Map<Path, CodeMap> map) {
        validationOutput.lines().forEach(line -> reportErrorLine(line, errorReporter, map));
    }

    /**
     * Reports the validation message contained in
     * {@code line} if such a message exists.
     * @param line a line of validator output
     * @param errorReporter an arbitrary ErrorReporter
     * @param maps a mapping from generated file paths to
     *             CodeMaps
     */
    private void reportErrorLine(String line, ErrorReporter errorReporter, Map<Path, CodeMap> maps) {
        Matcher matcher = p.matcher(line);
        if (matcher.matches()) {
            final Path path = Paths.get(matcher.group("path"));
            final Position generatedFilePosition = Position.fromOneBased(
                Integer.parseInt(matcher.group("line")), Integer.parseInt(matcher.group("column"))
            );
            final String message = String.format(
                "%s [%s:%s:%s]",
                matcher.group("message"), path.getFileName(),
                matcher.group("line"), matcher.group("column")
            );
            final CodeMap map = maps.get(path);
            final boolean isError = matcher.group("severity").toLowerCase().contains("error");
            if (map == null) {
                if (isError) {
                    errorReporter.reportError(message);
                } else {
                    errorReporter.reportWarning(message);
                }
                return;
            }
            for (Path srcFile : map.lfSourcePaths()) {
                // FIXME: Is it desirable for the error to be reported to every single LF file associated
                //  with the generated file containing the error? Or is it best to be more selective?
                Position lfFilePosition = map.adjusted(srcFile, generatedFilePosition);
                if (isError) {
                    errorReporter.reportError(srcFile, lfFilePosition.getOneBasedLine(), message);
                } else {
                    errorReporter.reportWarning(srcFile, lfFilePosition.getOneBasedLine(), message);
                }
            }
        }
    }
}
