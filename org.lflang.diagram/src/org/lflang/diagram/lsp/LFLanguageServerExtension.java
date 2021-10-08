package org.lflang.diagram.lsp;

import org.lflang.generator.IntegratedBuilder;
import org.lflang.LFStandaloneSetup;
import org.lflang.LFRuntimeModule;

import org.eclipse.emf.common.util.URI;
import org.eclipse.lsp4j.jsonrpc.services.JsonNotification;
import org.eclipse.xtext.ide.server.ILanguageServerExtension;
import org.eclipse.xtext.ide.server.ILanguageServerAccess;

class LFLanguageServerExtension implements ILanguageServerExtension {

    private static final IntegratedBuilder builder = new LFStandaloneSetup(new LFRuntimeModule()
        ).createInjectorAndDoEMFRegistration().getInstance(IntegratedBuilder.class);

    @Override
    public void initialize(ILanguageServerAccess access) {
        // Not implemented because this is not needed.
    }

    @JsonNotification("generator/build")
    public void build(String uri) {
        builder.run(URI.createURI(uri));
    }
}
