package org.lflang.generator.python;

import java.nio.file.Path;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.function.Function;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import org.eclipse.lsp4j.DiagnosticSeverity;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.generator.CodeMap;
import org.lflang.generator.DiagnosticReporting;
import org.lflang.generator.DiagnosticReporting.Strategy;
import org.lflang.generator.Position;
import org.lflang.generator.ValidationStrategy;
import org.lflang.util.LFCommand;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.google.common.collect.ImmutableMap;
import com.google.common.collect.ImmutableSet;

/**
 * A validator for generated Python.
 *
 * @author Peter Donovan
 */
public class PythonValidator extends org.lflang.generator.Validator {

    /** The pattern that diagnostics from the Python compiler typically follow. */
    private static final Pattern DIAGNOSTIC_MESSAGE_PATTERN = Pattern.compile(
        "(\\*\\*\\*)?\\s*File \"(?<path>.*?\\.py)\", line (?<line>\\d+)"
    );
    /** The pattern typically followed by the message that typically follows the main diagnostic line. */
    private static final Pattern MESSAGE = Pattern.compile("\\w*Error: .*");
    /** An alternative pattern that at least some diagnostics from the Python compiler may follow. */
    private static final Pattern ALT_DIAGNOSTIC_MESSAGE_PATTERN = Pattern.compile(
        ".*Error:.*line (?<line>\\d+)\\)"
    );

    /** The JSON parser. */
    private static final ObjectMapper mapper = new ObjectMapper()
        .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

    private final Set<String> protoNames;

    /**
     * The message format of Pylint's JSON output.
     */
    @SuppressWarnings( {"FieldCanBeLocal", "unused"})  // Unused fields are included for completeness.
    private static final class PylintMessage {
        private String type;
        private String module;
        private String obj;
        private Integer line;
        private Integer column;
        private Integer endLine;
        private Integer endColumn;
        private Path path;
        private String symbol;
        private String message;
        private String messageId;
        public void setType(String type) { this.type = type; }
        public void setModule(String module) { this.module = module; }
        public void setObj(String obj) { this.obj = obj; }
        public void setLine(int line) { this.line = line; }
        public void setColumn(int column) { this.column = column; }
        public void setEndLine(int endLine) { this.endLine = endLine; }
        public void setEndColumn(int endColumn) { this.endColumn = endColumn; }
        public void setPath(String path) { this.path = Path.of(path); }
        public void setSymbol(String symbol) { this.symbol = symbol; }
        public void setMessage(String message) { this.message = message; }
        @JsonProperty("message-id")
        public void setMessageId(String messageId) { this.messageId = messageId; }
        public Position getStart() {
            if (line != null && column != null) return Position.fromZeroBased(line - 1, column);
            // Use 0 as fallback for the column. This will cause bugs by taking some positions out of the line
            // adjuster's range.
            if (line != null) return Position.fromZeroBased(line - 1, 0);
            // This fallback will always fail with the line adjuster, but at least the program will not crash.
            return Position.ORIGIN;
        }
        public Position getEnd() {
            return endLine == null || endColumn == null ? getStart().plus(" ") :
                   Position.fromZeroBased(endLine - 1, endColumn);
        }
        public Path getPath(Path relativeTo) { return relativeTo.resolve(path); }
        public DiagnosticSeverity getSeverity() {
            // The following is consistent with VS Code's default behavior for pure Python:
            // https://code.visualstudio.com/docs/python/linting#_pylint
            switch (type.toLowerCase()) {
            case "refactor":
                return DiagnosticSeverity.Hint;
            case "warning":
                return DiagnosticSeverity.Warning;
            case "error":
            case "fatal":
                return DiagnosticSeverity.Error;
            case "convention":
            default:
                return DiagnosticSeverity.Information;
            }
        }
    }

    private static final Pattern PylintNoNamePattern = Pattern.compile("Instance of '(?<name>\\w+)' has no .*");

    private final FileConfig fileConfig;
    private final ErrorReporter errorReporter;
    private final ImmutableMap<Path, CodeMap> codeMaps;

    /**
     * Initialize a {@code PythonValidator} for a build process using {@code fileConfig} and
     * report errors to {@code errorReporter}.
     * @param fileConfig The file configuration of this build.
     * @param errorReporter The reporter to which diagnostics should be sent.
     * @param codeMaps A mapping from generated file paths to code maps that map them back to
     *                 LF sources.
     * @param protoNames The names of any protocol buffer message types that are used in the LF
     *                   program being built.
     */
    public PythonValidator(
        FileConfig fileConfig,
        ErrorReporter errorReporter,
        Map<Path,CodeMap> codeMaps,
        Set<String> protoNames
    ) {
        super(errorReporter, codeMaps);
        this.fileConfig = fileConfig;
        this.errorReporter = errorReporter;
        this.codeMaps = ImmutableMap.copyOf(codeMaps);
        this.protoNames = ImmutableSet.copyOf(protoNames);
    }

    @Override
    protected Collection<ValidationStrategy> getPossibleStrategies() { return List.of(
        new ValidationStrategy() {
            @Override
            public LFCommand getCommand(Path generatedFile) {
                return LFCommand.get(
                    "python3",
                    List.of("-c", "import compileall; compileall.compile_dir('.', quiet=1)"),
                    true,
                    fileConfig.getSrcGenPkgPath()
                );
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
                        if (!tryReportTypical(lines, i)) {
                            tryReportAlternative(lines, i);
                        }
                    }
                };
            }

            /**
             * Try to report a typical error message from the Python compiler.
             *
             * @param lines The lines of output from the compiler.
             * @param i     The current index at which a message may start. Guaranteed to be less
             *              than
             *              {@code lines.length - 3}.
             * @return Whether an error message was reported.
             */
            private boolean tryReportTypical(String[] lines, int i) {
                Matcher main = DIAGNOSTIC_MESSAGE_PATTERN.matcher(lines[i]);
                Matcher messageMatcher = MESSAGE.matcher(lines[i + 3]);
                String message = messageMatcher.matches() ? messageMatcher.group() : "Syntax Error";
                if (main.matches()) {
                    int line = Integer.parseInt(main.group("line"));
                    CodeMap map = codeMaps.get(fileConfig.getSrcGenPkgPath().resolve(Path.of(main.group("path"))).normalize());
                    Position genPosition = Position.fromOneBased(line, Integer.MAX_VALUE); // Column is just a placeholder.
                    if (map == null) {
                        errorReporter.report(null, DiagnosticSeverity.Error, message, 1);  // Undesirable fallback
                    } else {
                        for (Path lfFile : map.lfSourcePaths()) {
                            Position lfPosition = map.adjusted(lfFile, genPosition);
                            // TODO: We could be more precise than just getting the right line, but the way the output
                            //  is formatted (with leading whitespace possibly trimmed) does not make it easy.
                            errorReporter.report(lfFile, DiagnosticSeverity.Error, message, lfPosition.getOneBasedLine());
                        }
                    }
                    return true;
                }
                return false;
            }

            /**
             * Try to report an alternative error message from the Python compiler.
             *
             * @param lines The lines of output from the compiler.
             * @param i     The current index at which a message may start.
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
                                lfFile,
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
        },
        new ValidationStrategy() {
            @Override
            public LFCommand getCommand(Path generatedFile) {
                return LFCommand.get(
                    "pylint",
                    List.of("--output-format=json", generatedFile.getFileName().toString()),
                    true,
                    fileConfig.getSrcGenPath()
                );
            }

            @Override
            public Strategy getErrorReportingStrategy() {
                return (a, b, c) -> {};
            }

            @Override
            public Strategy getOutputReportingStrategy() {
                return (validationOutput, errorReporter, codeMaps) -> {
                    if (validationOutput.isBlank()) return;
                    try {
                        for (PylintMessage message : mapper.readValue(validationOutput, PylintMessage[].class)) {
                            if (shouldIgnore(message)) continue;
                            CodeMap map = codeMaps.get(message.getPath(fileConfig.getSrcGenPath()));
                            if (map != null) {
                                for (Path lfFile : map.lfSourcePaths()) {
                                    Function<Position, Position> adjust = p -> map.adjusted(lfFile, p);
                                    String humanMessage = DiagnosticReporting.messageOf(
                                        message.message,
                                        message.getPath(fileConfig.getSrcGenPath()),
                                        message.getStart()
                                    );
                                    Position lfStart = adjust.apply(message.getStart());
                                    Position lfEnd = adjust.apply(message.getEnd());
                                    bestEffortReport(
                                        errorReporter,
                                        adjust,
                                        lfStart,
                                        lfEnd,
                                        lfFile,
                                        message.getSeverity(),
                                        humanMessage
                                    );
                                }
                            }
                        }
                    } catch (JsonProcessingException e) {
                        System.err.printf("Failed to parse \"%s\":%n", validationOutput);
                        e.printStackTrace();
                        errorReporter.reportWarning(
                            "Failed to parse linter output. The Lingua Franca code generator is tested with Pylint "
                             + "version 2.12.2. Consider updating Pylint if you have an older version."
                        );
                    }
                };
            }

            /**
             * Return whether the given message should be ignored.
             * @param message A Pylint message that is a candidate to be reported.
             * @return whether {@code message} should be reported.
             */
            private boolean shouldIgnore(PylintMessage message) {
                // Code generation does not preserve whitespace, so this check is unreliable.
                if (message.symbol.equals("trailing-whitespace") || message.symbol.equals("line-too-long")) return true;
                // This filters out Pylint messages concerning missing members in types defined by protocol buffers.
                // FIXME: Make this unnecessary, perhaps using https://github.com/nelfin/pylint-protobuf.
                Matcher matcher = PylintNoNamePattern.matcher(message.message);
                return message.symbol.equals("no-member")
                        && matcher.matches() && protoNames.contains(matcher.group("name"));
            }

            /** Make a best-effort attempt to place the diagnostic on the correct line. */
            private void bestEffortReport(
                ErrorReporter errorReporter,
                Function<Position, Position> adjust,
                Position lfStart,
                Position lfEnd,
                Path file,
                DiagnosticSeverity severity,
                String humanMessage
            ) {
                if (!lfEnd.equals(Position.ORIGIN) && !lfStart.equals(Position.ORIGIN)) { // Ideal case
                    errorReporter.report(file, severity, humanMessage, lfStart, lfEnd);
                } else {  // Fallback: Try to report on the correct line, or failing that, just line 1.
                    if (lfStart.equals(Position.ORIGIN)) lfStart = adjust.apply(
                        Position.fromZeroBased(lfStart.getZeroBasedLine(), Integer.MAX_VALUE)
                    );
                    // FIXME: It might be better to improve style of generated code instead of quietly returning here.
                    if (lfStart.equals(Position.ORIGIN) && severity != DiagnosticSeverity.Error) return;
                    errorReporter.report(file, severity, humanMessage, lfStart.getOneBasedLine());
                }
            }

            @Override
            public boolean isFullBatch() {
                return false;
            }

            @Override
            public int getPriority() {
                return 1;
            }
        }
    ); }

    @Override
    protected Pair<Strategy, Strategy> getBuildReportingStrategies() {
        return new Pair<>((a, b, c) -> {}, (a, b, c) -> {});
    }
}
