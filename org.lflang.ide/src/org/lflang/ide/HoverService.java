package org.lflang.ide;

import java.util.ArrayList;
import java.util.List;

import org.eclipse.xtext.ide.server.hover.IHoverService;
import org.eclipse.lsp4j.Hover;
import org.eclipse.xtext.util.CancelIndicator;
import org.eclipse.xtext.ide.server.Document;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.lsp4j.jsonrpc.messages.Either;
import org.eclipse.lsp4j.MarkedString;
import org.eclipse.lsp4j.HoverParams;
import org.eclipse.lsp4j.services.LanguageServer;
import org.apache.log4j.Logger;

class HoverService implements IHoverService {

    private static final Logger LOG = Logger.getLogger(LanguageServer.class);

    @Override
    public Hover hover(Document document, XtextResource resource, HoverParams params, CancelIndicator cancelIndicator) {
        // TODO Move the following call to a more appropriate location
        LOG.debug("Hover detected.");
        DocumentRegistry.getInstance().refreshDocument(resource, document);
        // TODO Provide a real implementation
        final List<Either<String, MarkedString>> testHover = new ArrayList<>();
        testHover.add(Either.forLeft("This is a test hover provided by TestHoverService."));
        return new Hover(testHover);
    }
}