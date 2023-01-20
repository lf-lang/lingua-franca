package org.lflang.federated.generator;

import java.nio.file.Path;
import java.util.Map;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.lsp4j.DiagnosticSeverity;

import org.lflang.ErrorReporter;
import org.lflang.generator.CodeMap;
import org.lflang.generator.Position;
import org.lflang.generator.Range;

public class LineAdjustingErrorReporter implements ErrorReporter {

    private final ErrorReporter parent;
    private final Map<Path, CodeMap> codeMapMap;

    public LineAdjustingErrorReporter(ErrorReporter parent, Map<Path, CodeMap> codeMapMap) {
        this.parent = parent;
        this.codeMapMap = codeMapMap;
    }

    @Override
    public String reportError(String message) {
        return parent.reportError(message);
    }

    @Override
    public String reportWarning(String message) {
        return parent.reportWarning(message);
    }

    @Override
    public String reportInfo(String message) {
        return parent.reportInfo(message);
    }

    @Override
    public String reportError(EObject object, String message) {
        return parent.reportError(object, message);
    }

    @Override
    public String reportWarning(EObject object, String message) {
        return parent.reportWarning(object, message);
    }

    @Override
    public String reportInfo(EObject object, String message) {
        return parent.reportInfo(object, message);
    }

    @Override
    public String reportError(Path file, Integer line, String message) {
        return report(file, line, message, DiagnosticSeverity.Error);
    }

    @Override
    public String reportWarning(Path file, Integer line, String message) {
        return report(file, line, message, DiagnosticSeverity.Warning);
    }

    @Override
    public String reportInfo(Path file, Integer line, String message) {
        return report(file, line, message, DiagnosticSeverity.Information);
    }

    private String report(Path file, Integer line, String message, DiagnosticSeverity severity) {
        if (line == null) return report(file, severity, message);
        var position = Position.fromOneBased(line, Integer.MAX_VALUE);
        return report(
            file,
            severity,
            message,
            Position.fromZeroBased(position.getZeroBasedLine(), 0),
            position
        );
    }

    @Override
    public String report(Path file, DiagnosticSeverity severity, String message) {
        return ErrorReporter.super.report(file, severity, message);
    }

    @Override
    public String report(Path file, DiagnosticSeverity severity, String message, int line) {
        return ErrorReporter.super.report(file, severity, message, line);
    }

    @Override
    public String report(
        Path file,
        DiagnosticSeverity severity,
        String message,
        Position startPos,
        Position endPos
    ) {
        String ret = null;
        if (codeMapMap.containsKey(file)) {
            var relevantMap = codeMapMap.get(file);
            for (Path lfSource : relevantMap.lfSourcePaths()) {
                var adjustedRange = relevantMap.adjusted(
                    lfSource,
                    new Range(startPos, endPos)
                );
                ret = parent.report(
                    lfSource,
                    severity,
                    message,
                    adjustedRange.getStartInclusive().equals(Position.ORIGIN) ?
                        Position.fromZeroBased(
                            adjustedRange.getEndExclusive().getZeroBasedLine(),
                            0
                        ) : adjustedRange.getStartInclusive(),
                    adjustedRange.getEndExclusive()
                );
            }
        }
        if (ret == null) return severity == DiagnosticSeverity.Error ? reportError(message) : reportWarning(message);
        return ret;
    }

    @Override
    public boolean getErrorsOccurred() {
        return parent.getErrorsOccurred();
    }
}
