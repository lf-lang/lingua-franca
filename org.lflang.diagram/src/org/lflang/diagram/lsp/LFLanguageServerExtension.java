package org.lflang.diagram.lsp;

import java.util.concurrent.CompletableFuture;

import org.eclipse.emf.common.util.URI;
import org.eclipse.lsp4j.jsonrpc.services.JsonNotification;
import org.eclipse.lsp4j.jsonrpc.services.JsonRequest;
import org.eclipse.lsp4j.jsonrpc.CompletableFutures;
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

    /** The IntegratedBuilder instance that handles all build requests for the current session. */
    private static final IntegratedBuilder builder = new LFStandaloneSetup(new LFRuntimeModule())
        .createInjectorAndDoEMFRegistration().getInstance(IntegratedBuilder.class);

    @Override
    public void initialize(ILanguageServerAccess access) {
        // Not implemented because this is not needed.
    }

    /**
     * Handles a request for a complete build of the Lingua
     * Franca file specified by {@code uri}.
     * @param uri the URI of the LF file of interest
     */
    @JsonRequest("generator/build")
    public CompletableFuture<String> build(String uri) {
        return CompletableFutures.computeAsync(cancelToken -> builder.run(URI.createURI(uri), true).getUserMessage());
    }

    /**
     * Handles a request for the most complete build of the
     * specified Lingua Franca file that can be done in a
     * limited amount of time.
     * @param uri the URI of the LF file of interest
     */
    @JsonNotification("generator/partialBuild")
    public void partialBuild(String uri) {
        builder.run(URI.createURI(uri), false);
    }
}
