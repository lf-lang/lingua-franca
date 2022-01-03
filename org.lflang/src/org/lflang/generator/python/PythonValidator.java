package org.lflang.generator.python;

import java.nio.file.Path;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.eclipse.lsp4j.DiagnosticSeverity;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.generator.CodeMap;
import org.lflang.generator.DiagnosticReporting.Strategy;
import org.lflang.generator.Position;
import org.lflang.generator.ValidationStrategy;
import org.lflang.generator.Validator;
import org.lflang.util.LFCommand;

import com.google.common.collect.ImmutableMap;

public class PythonValidator extends Validator {

    /** The pattern that diagnostics from the Python compiler typically follow. */
    private static final Pattern DIAGNOSTIC_MESSAGE_PATTERN = Pattern.compile(
        "\\*\\*\\*\\s+File \"(?<path>.*?\\.py)\", line (?<line>\\d+)"
    );
    /** The pattern typically followed by the message that typically follows the main diagnostic line. */
    private static final Pattern MESSAGE = Pattern.compile("\\w*Error: .*");
    /** An alternative pattern that at least some diagnostics from the Python compiler may follow. */
    private static final Pattern ALT_DIAGNOSTIC_MESSAGE_PATTERN = Pattern.compile(
        "\\*\\*\\*.*Error:.*line (?<line>\\d+)\\)"
    );

    private final FileConfig fileConfig;
    private final ErrorReporter errorReporter;
    private final ImmutableMap<Path, CodeMap> codeMaps;

    public PythonValidator(FileConfig fileConfig, ErrorReporter errorReporter, Map<Path, CodeMap> codeMaps) {
        super(errorReporter, codeMaps);
        this.fileConfig = fileConfig;
        this.errorReporter = errorReporter;
        this.codeMaps = ImmutableMap.copyOf(codeMaps);
    }

    @Override
    protected Collection<ValidationStrategy> getPossibleStrategies() {
        return List.of(new ValidationStrategy() {
            @Override
            public LFCommand getCommand(Path generatedFile) {
                return LFCommand.get("python3", List.of("-m", "compileall"), fileConfig.getSrcGenPath());
            }

            @Override
            public Strategy getErrorReportingStrategy() {
                return (a, b, c) -> {};
            }

            @Override
            public Strategy getOutputReportingStrategy() {
                return (String validationOutput, ErrorReporter errorReporter, Map<Path, CodeMap> map) -> {
                    String[] lines = (validationOutput + "\n\n\n").lines().toArray(String[]::new);
                    for (int i = 0; i < lines.length - 3; i++) {
                        if (!tryReportTypical(lines, i)) tryReportAlternative(lines, i);
                    }
                };
            }

            /**
             * Try to report a typical error message from the Python compiler.
             * @param lines The lines of output from the compiler.
             * @param i The current index at which a message may start. Guaranteed to be less than
             *          {@code lines.length - 3}.
             * @return Whether an error message was reported.
             */
            private boolean tryReportTypical(String[] lines, int i) {
                Matcher main = DIAGNOSTIC_MESSAGE_PATTERN.matcher(lines[i]);
                Matcher messageMatcher = MESSAGE.matcher(lines[i + 3]);
                String message = messageMatcher.matches() ? messageMatcher.group() : "Syntax Error";
                if (main.matches()) {
                    int line = Integer.parseInt(main.group("line"));
                    CodeMap map = codeMaps.get(Path.of(main.group("path")));
                    Position genPosition = Position.fromOneBased(line, Integer.MAX_VALUE); // Column is just a placeholder.
                    if (map == null) {
                        errorReporter.report(DiagnosticSeverity.Error, message, 1);  // Undesirable fallback
                    } else {
                        for (Path lfFile : map.lfSourcePaths()) {
                            Position lfPosition = map.adjusted(lfFile, genPosition);
                            errorReporter.report(DiagnosticSeverity.Error, message, lfPosition.getOneBasedLine());
                        }
                    }
                    return true;
                }
                return false;
            }

            /**
             * Try to report an alternative error message from the Python compiler.
             * @param lines The lines of output from the compiler.
             * @param i The current index at which a message may start.
             */
            private void tryReportAlternative(String[] lines, int i) {
                Matcher main = ALT_DIAGNOSTIC_MESSAGE_PATTERN.matcher(lines[i]);
                if (main.matches()) {
                    int line = Integer.parseInt(main.group("line"));
                    Iterable<CodeMap> relevantMaps = codeMaps.keySet().stream()
                        .filter(p -> main.group().contains(p.getFileName().toString()))
                        .map(codeMaps::get)::iterator;
                    for (CodeMap map : relevantMaps) {  // There should almost always be exactly one of these
                        for (Path lfFile : map.lfSourcePaths()) {
                            errorReporter.report(
                                DiagnosticSeverity.Error,
                                main.group().replace("*** ", "").replace("Sorry: ", ""),
                                map.adjusted(lfFile, Position.fromOneBased(line, 1)).getOneBasedLine()
                            );
                        }
                    }
                }
            }

            @Override
            public boolean isFullBatch() {
                return true;
            }

            @Override
            public int getPriority() {
                return 0;
            }
        });
    }

    @Override
    protected Pair<Strategy, Strategy> getBuildReportingStrategies() {
        return new Pair<>((a, b, c) -> {}, (a, b, c) -> {});
    }
}
