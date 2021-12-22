package org.lflang.diagram.lsp;

import java.nio.file.Paths;
import java.util.concurrent.CompletableFuture;
import java.util.function.Function;

import org.eclipse.emf.common.util.URI;
import org.eclipse.lsp4j.jsonrpc.services.JsonNotification;
import org.eclipse.lsp4j.jsonrpc.services.JsonRequest;
import org.eclipse.lsp4j.jsonrpc.CancelChecker;
import org.eclipse.lsp4j.jsonrpc.CompletableFutures;
import org.eclipse.lsp4j.ProgressParams;
import org.eclipse.lsp4j.services.LanguageClient;
import org.eclipse.xtext.ide.server.ILanguageServerExtension;
import org.eclipse.xtext.ide.server.ILanguageServerAccess;
import org.eclipse.xtext.util.CancelIndicator;

import org.lflang.generator.IntegratedBuilder;
import org.lflang.LFStandaloneSetup;
import org.lflang.LFRuntimeModule;

/**
 * Provides Lingua-Franca-specific extensions to the
 * language server's behavior.
 * @author Peter Donovan <peterdonovan@berkeley.edu>
 */
class LFLanguageServerExtension implements ILanguageServerExtension {

    /**
     * Describes a build process that has a progress.
     */
    private static class BuildWithProgress implements Function<CancelChecker, String> {
        private final Progress progress;
        private final boolean mustComplete;
        private final String uri;
        private final CancelIndicator cancelIndicator;

        /**
         * Instantiates the building of the file located at {@code uri}.
         * @param uri The location of the file to be built.
         * @param mustComplete Whether the build process must be brought to
         * completion.
         */
        public BuildWithProgress(LanguageClient client, String uri, boolean mustComplete) {
            this.uri = uri;
            this.progress = new Progress(client, "Build \"" + Paths.get(uri).getFileName() + "\"");
            this.cancelIndicator = progress.getCancelIndicator();
            this.mustComplete = mustComplete;
        }

        /**
         * Executes {@code this}.
         * @param cancelChecker A useless parameter.
         */
        public String apply(CancelChecker cancelChecker) {
            // cancelChecker is ignored because the framework appears to provide it but not use it.
            progress.begin();
            String message = builder.run(
                URI.createURI(uri), mustComplete, progress::report, cancelIndicator
            ).getUserMessage();
            progress.end(message);
            return message;
        }
    }

    /** The IntegratedBuilder instance that handles all build requests for the current session. */
    private static final IntegratedBuilder builder = new LFStandaloneSetup(new LFRuntimeModule())
        .createInjectorAndDoEMFRegistration().getInstance(IntegratedBuilder.class);

    /** The access point for reading documents, communicating with the language client, etc. */
    private LanguageClient client;

    @Override
    public void initialize(ILanguageServerAccess access) {
        // This method is never invoked. It might be useful if it were invoked, but seemingly that
        //  is just not how the framework works right now.
    }

    public void setClient(LanguageClient client) {
        this.client = client;
    }

    /**
     * Handles a request for a complete build of the Lingua
     * Franca file specified by {@code uri}.
     * @param uri the URI of the LF file of interest
     * @return A message describing the outcome of the build
     * process.
     */
    @JsonRequest("generator/build")
    public CompletableFuture<String> build(String uri) {
        if (client == null) return CompletableFuture.completedFuture(
            "Please wait for the Lingua Franca language server to be fully initialized."
        );
        return CompletableFutures.computeAsync(new BuildWithProgress(client, uri, true));
    }

    /**
     * Handles a request for the most complete build of the
     * specified Lingua Franca file that can be done in a
     * limited amount of time.
     * @param uri the URI of the LF file of interest
     */
    @JsonNotification("generator/partialBuild")
    public void partialBuild(String uri) {
        if (client == null) return;
        new BuildWithProgress(client, uri, false).apply(null);
    }
}
