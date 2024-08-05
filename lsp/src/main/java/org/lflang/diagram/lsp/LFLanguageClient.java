package org.lflang.diagram.lsp;

import de.cau.cs.kieler.klighd.lsp.KGraphLanguageClient;
import org.eclipse.lsp4j.jsonrpc.services.JsonNotification;
import org.eclipse.lsp4j.services.LanguageClient;

public interface LFLanguageClient extends KGraphLanguageClient, LanguageClient {

  @JsonNotification("notify/sendLibraryReactors")
  public void sendLibraryReactors(LibraryFileTree libraryFileTree);
}
