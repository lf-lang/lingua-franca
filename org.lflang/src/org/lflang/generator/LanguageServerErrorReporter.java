package org.lflang.generator;

import java.nio.file.Path;
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
    /** Whether errors occurred since the last reset. */
    boolean errorsOccurred = false;

    /* ------------------------  CONSTRUCTORS  -------------------------- */

    /**
     * Initializes a <code>DiagnosticAcceptor</code> for the
     * document whose parse tree root node is
     * <code>parseRoot</code>.
     * @param parseRoot the root of the AST of the document
     *                  for which this is an error reporter
     */
    public LanguageServerErrorReporter(EObject parseRoot) {
        this.parseRoot = parseRoot;
    }

    /* -----------------------  PUBLIC METHODS  ------------------------- */

    @Override
    public String reportError(String message) {
        return acceptDiagnostic(DiagnosticSeverity.Error, message);
    }

    @Override
    public String reportWarning(String message) {
        return acceptDiagnostic(DiagnosticSeverity.Warning, message);
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
        return acceptDiagnostic(DiagnosticSeverity.Error, message, line != null ? line : 0);
    }

    @Override
    public String reportWarning(Path file, Integer line, String message) {
        return acceptDiagnostic(DiagnosticSeverity.Warning, message, line != null ? line : 0);
    }

    @Override
    public boolean getErrorsOccurred() {
        return errorsOccurred;
    }

    @Override
    public void reset() {
        errorsOccurred = false;
    }

    public static void setClient(LanguageClient client) {
        LanguageServerErrorReporter.client = client;
    }

    /* -----------------------  PRIVATE METHODS  ------------------------ */

    /**
     * Reports a message of severity <code>severity</code>.
     * @param severity the severity of the message
     * @param message the message to send to the IDE
     * @return a string that describes the diagnostic
     */
    private String acceptDiagnostic(DiagnosticSeverity severity, String message) {
        return acceptDiagnostic(severity, message, 0);
    }

    /**
     * Reports a message of severity <code>severity</code>.
     * @param severity the severity of the message
     * @param message the message to send to the IDE
     * @param line the zero-based line number associated
     *             with the message
     * @return a string that describes the diagnostic
     */
    private String acceptDiagnostic(DiagnosticSeverity severity, String message, int line) {
        Optional<String> firstLine = getLine(line);
        return acceptDiagnostic(
            severity,
            message,
            Position.fromZeroBased(0, 0),
            Position.fromZeroBased(0, firstLine.isEmpty() ? 1 : firstLine.get().length())
        );
    }

    /**
     * Reports a message of severity <code>severity</code>.
     * @param severity the severity of the message
     * @param message the message to send to the IDE
     * @param startPos the position of the first character
     *                 of the range of interest
     * @param endPos the position immediately AFTER the
     *               final character of the range of
     *               interest
     * @return a string that describes the diagnostic
     */
    private String acceptDiagnostic(DiagnosticSeverity severity, String message, Position startPos, Position endPos) {
        // FIXME: Diagnostics should be collected, then sent all at once. That is why publishDiagnostics accepts a list.
        if (client != null)
            publishDiagnostics(List.of(new Diagnostic(
                toRange(startPos, endPos), message, severity, "Lingua Franca Language Server"
            )));
        if (severity == DiagnosticSeverity.Error) errorsOccurred = true;
        return "" + severity + ": " + message;
    }

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
     * Returns the line at index <code>line</code> in the
     * document for which this is an error reporter.
     * @param line the zero-based line index
     * @return the line located at the given index
     */
    private Optional<String> getLine(int line) {
        return getText().lines().skip(line).findFirst();
    }

    private void publishDiagnostics(List<Diagnostic> diagnostics) {
        PublishDiagnosticsParams publishDiagnosticsParams = new PublishDiagnosticsParams();
        publishDiagnosticsParams.setUri(parseRoot.eResource().getURI().toString());
        publishDiagnosticsParams.setDiagnostics(diagnostics);
        client.publishDiagnostics(publishDiagnosticsParams);
    }

    private Range toRange(Position p0, Position p1) {
        return new Range(
            new org.eclipse.lsp4j.Position(p0.getZeroBasedLine(), p0.getZeroBasedColumn()),
            new org.eclipse.lsp4j.Position(p1.getZeroBasedLine(), p1.getZeroBasedColumn())
        );
    }
}
