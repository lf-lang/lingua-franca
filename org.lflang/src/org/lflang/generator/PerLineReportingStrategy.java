package org.lflang.generator;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.jetbrains.annotations.Nullable;

import org.lflang.ErrorReporter;

public class PerLineReportingStrategy implements CommandErrorReportingStrategy {

    private final Pattern p;

    public PerLineReportingStrategy(Pattern p) {
        for (String groupName : new String[]{"path", "line", "column", "message"}) {
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
            if (map == null) {
                errorReporter.reportError(message);
                return;
            }
            for (Path srcFile : map.lfSourcePaths()) {
                // FIXME: Is it desirable for the error to be reported to every single LF file associated
                //  with the generated file containing the error? Or is it best to be more selective?
                Position lfFilePosition = map.adjusted(srcFile, generatedFilePosition);
                errorReporter.reportError(srcFile, lfFilePosition.getOneBasedLine(), message);
            }
        }
    }
}
