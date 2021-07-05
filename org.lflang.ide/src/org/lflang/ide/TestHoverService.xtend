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

class TestHoverService /*implements IHoverService*/ {
	
	/*override hover(Document document, XtextResource resource, HoverParams params, CancelIndicator cancelIndicator) {
		val testHover = new ArrayList<Either<String, MarkedString>>()
		testHover.add(Either.forLeft('This is a test hover provided by TestHoverService.'))
		return new Hover(testHover);
	}*/
	
}