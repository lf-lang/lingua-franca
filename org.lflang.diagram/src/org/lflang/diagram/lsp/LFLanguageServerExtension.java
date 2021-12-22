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
import org.eclipse.lsp4j.WorkDoneProgressCreateParams;
import org.eclipse.lsp4j.WorkDoneProgressBegin;
import org.eclipse.lsp4j.WorkDoneProgressEnd;
import org.eclipse.lsp4j.WorkDoneProgressNotification;
import org.eclipse.lsp4j.WorkDoneProgressReport;
import org.eclipse.lsp4j.jsonrpc.messages.Either;
import org.eclipse.xtext.ide.server.ILanguageServerExtension;
import org.eclipse.xtext.ide.server.ILanguageServerAccess;

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
     * Reports the progress of an ongoing task.
     */
    private static class Progress {
        private static int nextToken = 0;
        private final String title;
        private final int token;
        private final LanguageClient client;
        public Progress(LanguageClient client, String title) {
            this.client = client;
            this.token = nextToken++;
            this.title = title;
            client.createProgress(new WorkDoneProgressCreateParams(Either.forRight(token)));
        }

        /**
         * Reports that the task tracked by {@code this} is done.
         */
        public void begin() {
            WorkDoneProgressBegin begin = new WorkDoneProgressBegin();
            begin.setTitle(title);
            notifyProgress(begin);
        }

        /**
         * Reports the progress of the task tracked by {@code this}.
         * @param message A message describing the progress of the task.
         */
        public void report(String message) {
            // It is possible to report percent completion, but we choose not to support this feature (yet) because the
            // editor we currently target (VS Code) only supports the feature if the progress bar's location is
            // set to "ProgressLocation.Notification".
            WorkDoneProgressReport report = new WorkDoneProgressReport();
            report.setMessage(message);
            notifyProgress(report);
        }

        /**
         * Marks the task tracked by {@code this} as terminated.
         * @param message A message describing the outcome of the task.
         */
        public void end(String message) {
            WorkDoneProgressEnd end = new WorkDoneProgressEnd();
            end.setMessage(message);
            notifyProgress(end);
        }

        /**
         * Sends the given progress notification to the client.
         * @param notification
         */
        private void notifyProgress(WorkDoneProgressNotification notification) {
            client.notifyProgress(new ProgressParams(Either.forRight(token), notification));
        }
    }

    /**
     * Describes a build process that has a progress.
     */
    private static class BuildWithProgress implements Function<CancelChecker, String> {
        private final Progress progress;
        private final boolean mustComplete;
        private final String uri;

        /**
         * Instantiates the building of the file located at {@code uri}.
         * @param uri The location of the file to be built.
         * @param mustComplete Whether the build process must be brought to
         * completion.
         */
        public BuildWithProgress(LanguageClient client, String uri, boolean mustComplete) {
            this.uri = uri;
            this.progress = new Progress(client, "Build \"" + Paths.get(uri).getFileName() + "\"");
            this.mustComplete = mustComplete;
        }

        /**
         * Executes {@code this}.
         */
        public String apply(CancelChecker cancelToken) {
            progress.begin();
            String message = builder.run(URI.createURI(uri), mustComplete, progress::report).getUserMessage();
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
        // This method is never invoked. It might be useful if it were invoked, but apparently that
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
