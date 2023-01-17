package org.lflang.federated.generator;

import java.nio.file.Path;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.lsp4j.DiagnosticSeverity;

import org.lflang.ErrorReporter;
import org.lflang.generator.Position;

public class SynchronizedErrorReporter implements ErrorReporter {

    private final ErrorReporter parent;

    public SynchronizedErrorReporter(ErrorReporter parent) {
        this.parent = parent;
    }

    @Override
    public synchronized String reportError(String message) {
        return parent.reportError(message);
    }

    @Override
    public synchronized String reportWarning(String message) {
        return parent.reportWarning(message);
    }

    @Override
    public synchronized String reportInfo(String message) {
        return parent.reportInfo(message);
    }

    @Override
    public synchronized String reportError(EObject object, String message) {
        return parent.reportError(object, message);
    }

    @Override
    public synchronized String reportWarning(EObject object, String message) {
        return parent.reportWarning(object, message);
    }

    @Override
    public synchronized String reportInfo(EObject object, String message) {
        return parent.reportInfo(object, message);
    }

    @Override
    public synchronized String reportError(Path file, Integer line, String message) {
        return parent.reportError(file, line, message);
    }

    @Override
    public synchronized String reportWarning(Path file, Integer line, String message) {
        return parent.reportWarning(file, line, message);
    }

    @Override
    public synchronized String reportInfo(Path file, Integer line, String message) {
        return parent.reportInfo(file, line, message);
    }

    @Override
    public synchronized String report(Path file, DiagnosticSeverity severity, String message) {
        return parent.report(file, severity, message);
    }

    @Override
    public synchronized String report(Path file, DiagnosticSeverity severity, String message, int line) {
        return parent.report(file, severity, message, line);
    }

    @Override
    public synchronized String report(Path file, DiagnosticSeverity severity, String message, Position startPos, Position endPos) {
        return parent.report(file, severity, message, startPos, endPos);
    }

    @Override
    public synchronized boolean getErrorsOccurred() {
        return parent.getErrorsOccurred();
    }
}
