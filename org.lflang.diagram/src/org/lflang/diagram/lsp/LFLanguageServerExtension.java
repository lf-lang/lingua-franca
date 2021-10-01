package org.lflang.diagram.lsp;

import org.eclipse.lsp4j.jsonrpc.services.JsonNotification;
import org.eclipse.xtext.ide.server.ILanguageServerExtension;
import org.eclipse.xtext.ide.server.ILanguageServerAccess;

class LFLanguageServerExtension implements ILanguageServerExtension {
    @Override
    public void initialize(ILanguageServerAccess access) {
        // Not implemented because this is not needed.
    }

    @JsonNotification("generator/build")
    public void build(String uri) {
        // TODO: Implement
    }
}
