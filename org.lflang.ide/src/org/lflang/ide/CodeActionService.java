package org.lflang.ide;

import java.util.List;
import java.util.concurrent.CompletableFuture;

import org.apache.log4j.Logger;
import org.eclipse.lsp4j.CodeAction;
import org.eclipse.lsp4j.Command;
import org.eclipse.lsp4j.jsonrpc.messages.Either;
import org.eclipse.lsp4j.services.LanguageServer;
import org.eclipse.xtext.ide.server.codeActions.ICodeActionService2;

public class CodeActionService implements ICodeActionService2 {

    private static final Logger LOG = Logger.getLogger(LanguageServer.class);

    @Override
    public List<Either<Command, CodeAction>> getCodeActions(Options options) {
        LOG.debug("Getting code actions for document " + options.getResource().getURI());
        CompletableFuture.runAsync(() -> {
            DocumentRegistry.getInstance().refreshDocument(options.getResource(), options.getDocument());
        });
        return null;
    }
}
