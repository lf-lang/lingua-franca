package org.lflang.diagram.lsp;

import org.lflang.generator.IntegratedBuilder;
import org.lflang.LFStandaloneSetup;
import org.lflang.LFRuntimeModule;

import org.eclipse.emf.common.util.URI;
import org.eclipse.lsp4j.jsonrpc.services.JsonNotification;
import org.eclipse.xtext.ide.server.ILanguageServerExtension;
import org.eclipse.xtext.ide.server.ILanguageServerAccess;

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
    @JsonNotification("generator/build")
    public void build(String uri) {
        builder.run(URI.createURI(uri), true);
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
