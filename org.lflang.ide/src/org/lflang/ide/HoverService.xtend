package org.lflang.ide

import java.util.ArrayList;

import org.eclipse.xtext.ide.server.hover.IHoverService
import org.eclipse.lsp4j.Hover
import org.eclipse.xtext.util.CancelIndicator
import org.eclipse.xtext.ide.server.Document
import org.eclipse.xtext.resource.XtextResource
import org.eclipse.lsp4j.jsonrpc.messages.Either
import org.eclipse.lsp4j.MarkedString
import org.eclipse.lsp4j.HoverParams
import org.eclipse.lsp4j.services.LanguageServer
import org.apache.log4j.Logger
import java.io.File

class HoverService implements IHoverService {
	
	static val LOG = Logger.getLogger(LanguageServer)
	
	override hover(Document document, XtextResource resource, HoverParams params, CancelIndicator cancelIndicator) {
		// TODO Move the following call to a more appropriate location
		DocumentRegistry.getInstance().refreshDocument(resource, document)
		// TODO Provide a real implementation
		val testHover = new ArrayList<Either<String, MarkedString>>()
		testHover.add(Either.forLeft('This is a test hover provided by TestHoverService.'))
		return new Hover(testHover);
	}
}