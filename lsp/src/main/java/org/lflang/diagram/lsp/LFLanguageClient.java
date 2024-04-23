package org.lflang.diagram.lsp;

import org.eclipse.lsp4j.jsonrpc.services.JsonNotification;
import org.eclipse.lsp4j.services.LanguageClient;

import de.cau.cs.kieler.klighd.lsp.KGraphLanguageClient;

public interface LFLanguageClient extends KGraphLanguageClient, LanguageClient {

    @JsonNotification("update/example")
    public void test(String updateMessage);

}
