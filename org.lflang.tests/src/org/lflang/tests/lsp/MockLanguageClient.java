package org.lflang.tests.lsp;

import java.util.Collections;
import java.util.List;
import java.util.concurrent.CompletableFuture;

import org.eclipse.lsp4j.Diagnostic;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.eclipse.lsp4j.MessageActionItem;
import org.eclipse.lsp4j.MessageParams;
import org.eclipse.lsp4j.PublishDiagnosticsParams;
import org.eclipse.lsp4j.ShowMessageRequestParams;
import org.eclipse.lsp4j.services.LanguageClient;

public class MockLanguageClient implements LanguageClient {

    private List<Diagnostic> receivedDiagnostics = List.of();

    @Override
    public void telemetryEvent(Object object) {
        // Do nothing.
    }

    @Override
    public void publishDiagnostics(PublishDiagnosticsParams diagnostics) {
        receivedDiagnostics = diagnostics.getDiagnostics();
        for (Diagnostic d : diagnostics.getDiagnostics()) {
            (
                (d.getSeverity() == DiagnosticSeverity.Error || d.getSeverity() == DiagnosticSeverity.Warning) ?
                System.err : System.out
            ).println("Test client received diagnostic: " + d.getMessage());
        }
    }

    @Override
    public void showMessage(MessageParams messageParams) {
        System.out.println("Test client received message: " + messageParams.getMessage());
    }

    @Override
    public CompletableFuture<MessageActionItem> showMessageRequest(ShowMessageRequestParams requestParams) {
        showMessage(requestParams);
        return null;
    }

    @Override
    public void logMessage(MessageParams message) {
        showMessage(message);
    }

    /** Return the diagnostics that {@code this} has received. */
    public List<Diagnostic> getReceivedDiagnostics() {
        return Collections.unmodifiableList(receivedDiagnostics);
    }

    /** Clear the diagnostics recorded by {@code this}. */
    public void clearDiagnostics() {
        receivedDiagnostics = List.of();
    }
}
