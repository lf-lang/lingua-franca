package org.lflang.generator;

import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.eclipse.emf.common.util.URI;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.lsp4j.Diagnostic;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.eclipse.lsp4j.PublishDiagnosticsParams;
import org.eclipse.lsp4j.Range;
import org.eclipse.xtext.nodemodel.util.NodeModelUtils;
import org.eclipse.lsp4j.services.LanguageClient;

import org.lflang.ErrorReporter;

/**
 * Report diagnostics to the language client.
 *
 * @author Peter Donovan
 */
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
    private final Map<Path, List<Diagnostic>> diagnostics;

    /* ------------------------  CONSTRUCTORS  -------------------------- */

    /**
     * Initialize a {@code DiagnosticAcceptor} for the
     * document whose parse tree root node is
     * {@code parseRoot}.
     * @param parseRoot the root of the AST of the document
     *                  for which this is an error reporter
     */
    public LanguageServerErrorReporter(EObject parseRoot) {
        this.parseRoot = parseRoot;
        this.diagnostics = new HashMap<>();
    }

    /* -----------------------  PUBLIC METHODS  ------------------------- */

    @Override
    public String reportError(String message) {
        return report(getMainFile(), DiagnosticSeverity.Error, message);
    }

    @Override
    public String reportWarning(String message) {
        return report(getMainFile(), DiagnosticSeverity.Warning, message);
    }

    @Override
    public String reportInfo(String message) {
        return report(getMainFile(), DiagnosticSeverity.Information, message);
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
    public String reportInfo(EObject object, String message) {
        return reportInfo(message);
    }

    @Override
    public String reportError(Path file, Integer line, String message) {
        return report(file, DiagnosticSeverity.Error, message, line != null ? line : 1);
    }

    @Override
    public String reportWarning(Path file, Integer line, String message) {
        return report(file, DiagnosticSeverity.Warning, message, line != null ? line : 1);
    }

    @Override
    public String reportInfo(Path file, Integer line, String message) {
        return report(file, DiagnosticSeverity.Information, message, line != null ? line : 1);
    }

    @Override
    public boolean getErrorsOccurred() {
        return diagnostics.values().stream().anyMatch(
            it -> it.stream().anyMatch(diagnostic -> diagnostic.getSeverity() == DiagnosticSeverity.Error)
        );
    }

    @Override
    public String report(Path file, DiagnosticSeverity severity, String message) {
        return report(file, severity, message, 1);
    }

    @Override
    public String report(Path file, DiagnosticSeverity severity, String message, int line) {
        Optional<String> text = getLine(line - 1);
        return report(
            file,
            severity,
            message,
            Position.fromOneBased(line, 1),
            Position.fromOneBased(line, 1 + (text.isEmpty() ? 0 : text.get().length()))
        );
    }

    @Override
    public String report(Path file, DiagnosticSeverity severity, String message, Position startPos, Position endPos) {
        if (file == null) file = getMainFile();
        diagnostics.putIfAbsent(file, new ArrayList<>());
        diagnostics.get(file).add(new Diagnostic(
            toRange(startPos, endPos), message, severity, "LF Language Server"
        ));
        return "" + severity + ": " + message;
    }

    /**
     * Save a reference to the language client.
     * @param client the language client
     */
    public static void setClient(LanguageClient client) {
        LanguageServerErrorReporter.client = client;
    }

    /**
     * Publish diagnostics by forwarding them to the
     * language client.
     */
    public void publishDiagnostics() {
        if (client == null) {
            System.err.println(
                "WARNING: Cannot publish diagnostics because the language client has not yet been found."
            );
            return;
        }
        for (Path file : diagnostics.keySet()) {
            PublishDiagnosticsParams publishDiagnosticsParams = new PublishDiagnosticsParams();
            publishDiagnosticsParams.setUri(URI.createFileURI(file.toString()).toString());
            publishDiagnosticsParams.setDiagnostics(diagnostics.get(file));
            client.publishDiagnostics(publishDiagnosticsParams);
        }
    }

    /* -----------------------  PRIVATE METHODS  ------------------------ */

    /** Return the file on which the current validation process was triggered. */
    private Path getMainFile() {
        return Path.of(parseRoot.eResource().getURI().toFileString());
    }

    /**
     * Return the text of the document for which this is an
     * error reporter.
     * @return the text of the document for which this is an
     * error reporter
     */
    private String getText() {
        return NodeModelUtils.getNode(parseRoot).getText();
    }

    /**
     * Return the line at index {@code line} in the
     * document for which this is an error reporter.
     * @param line the zero-based line index
     * @return the line located at the given index
     */
    private Optional<String> getLine(int line) {
        return getText().lines().skip(line).findFirst();
    }

    /**
     * Return the Range that starts at {@code p0} and ends
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
