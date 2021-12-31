package org.lflang.generator;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.eclipse.emf.ecore.EObject;
import org.eclipse.lsp4j.Diagnostic;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.eclipse.lsp4j.PublishDiagnosticsParams;
import org.eclipse.lsp4j.Range;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.eclipse.lsp4j.services.LanguageClient;

import org.lflang.ErrorReporter;

public class LanguageServerErrorReporter implements ErrorReporter {

    /**
     * The language client to which errors should be
     * reported, if such a client is available.
     * FIXME: This is a de facto global, and it is a hack.
     */
    private static LanguageClient client;

    /** The document for which this is a diagnostic acceptor. */
    private final EObject parseRoot;
    /** The list of all diagnostics since the last reset. */
    private final List<Diagnostic> diagnostics;

    /* ------------------------  CONSTRUCTORS  -------------------------- */

    /**
     * Initializes a {@code DiagnosticAcceptor} for the
     * document whose parse tree root node is
     * {@code parseRoot}.
     * @param parseRoot the root of the AST of the document
     *                  for which this is an error reporter
     */
    public LanguageServerErrorReporter(EObject parseRoot) {
        this.parseRoot = parseRoot;
        this.diagnostics = new ArrayList<>();
    }

    /* -----------------------  PUBLIC METHODS  ------------------------- */

    @Override
    public String reportError(String message) {
        return report(DiagnosticSeverity.Error, message);
    }

    @Override
    public String reportWarning(String message) {
        return report(DiagnosticSeverity.Warning, message);
    }

    @Override
    public String reportError(EObject object, String message) {
        return reportError(message);
    }

    @Override
    public String reportWarning(EObject object, String message) {
        return reportWarning(message);
    }

    @Override
    public String reportError(Path file, Integer line, String message) {
        return report(DiagnosticSeverity.Error, message, line != null ? line - 1 : 0);
    }

    @Override
    public String reportWarning(Path file, Integer line, String message) {
        return report(DiagnosticSeverity.Warning, message, line != null ? line - 1 : 0);
    }

    @Override
    public boolean getErrorsOccurred() {
        return diagnostics.stream().anyMatch(diagnostic -> diagnostic.getSeverity() == DiagnosticSeverity.Error);
    }

    @Override
    public String report(DiagnosticSeverity severity, String message) {
        return report(severity, message, 0);
    }

    @Override
    public String report(DiagnosticSeverity severity, String message, int line) {
        Optional<String> text = getLine(line);
        return report(
            severity,
            message,
            Position.fromZeroBased(line, 0),
            Position.fromZeroBased(line, text.isEmpty() ? 1 : text.get().length())
        );
    }

    @Override
    public String report(DiagnosticSeverity severity, String message, Position startPos, Position endPos) {
        diagnostics.add(new Diagnostic(
            toRange(startPos, endPos), message, severity, "LF Language Server"
        ));
        return "" + severity + ": " + message;
    }

    /**
     * Saves a reference to the language client.
     * @param client the language client
     */
    public static void setClient(LanguageClient client) {
        LanguageServerErrorReporter.client = client;
    }

    /**
     * Publishes diagnostics by forwarding them to the
     * language client.
     */
    public void publishDiagnostics() {
        if (client == null) {
            System.err.println(
                "WARNING: Cannot publish diagnostics because the language client has not yet been found."
            );
            return;
        }
        PublishDiagnosticsParams publishDiagnosticsParams = new PublishDiagnosticsParams();
        publishDiagnosticsParams.setUri(parseRoot.eResource().getURI().toString());
        publishDiagnosticsParams.setDiagnostics(diagnostics);
        client.publishDiagnostics(publishDiagnosticsParams);
    }

    /* -----------------------  PRIVATE METHODS  ------------------------ */

    /**
     * Returns the text of the document for which this is an
     * error reporter.
     * @return the text of the document for which this is an
     * error reporter
     */
    private String getText() {
        return NodeModelUtils.getNode(parseRoot).getText();
    }

    /**
     * Returns the line at index {@code line} in the
     * document for which this is an error reporter.
     * @param line the zero-based line index
     * @return the line located at the given index
     */
    private Optional<String> getLine(int line) {
        return getText().lines().skip(line).findFirst();
    }

    /**
     * Returns the Range that starts at {@code p0} and ends
     * at {@code p1}.
     * @param p0 an arbitrary Position
     * @param p1 a Position that is greater than {@code p0}
     * @return the Range that starts at {@code p0} and ends
     * at {@code p1}
     */
    private Range toRange(Position p0, Position p1) {
        return new Range(
            new org.eclipse.lsp4j.Position(p0.getZeroBasedLine(), p0.getZeroBasedColumn()),
            new org.eclipse.lsp4j.Position(p1.getZeroBasedLine(), p1.getZeroBasedColumn())
        );
    }
}
