package org.lflang.ide

import java.util.ArrayList;

import org.eclipse.xtext.ide.server.rename.IRenameService2
import org.eclipse.xtext.ide.server.rename.IRenameService2.PrepareRenameOptions
import org.eclipse.xtext.ide.server.rename.IRenameService2.Options
import org.eclipse.lsp4j.jsonrpc.messages.Either
import org.eclipse.lsp4j.Range
import org.eclipse.lsp4j.Position

class TestRenameService implements IRenameService2 {
	
	override prepareRename(PrepareRenameOptions options) {
		System.out.println("DEBUG: in prepareRename");
		return Either.forLeft(new Range(
			new Position(0, 0),
			new Position(0, 10)
		));
	}
	
	override rename(Options options) {
		return null;
	}
	
	
}