package org.lflang.ide;

import org.lflang.ide.document.LFDocument;

import org.apache.log4j.Logger;

import org.eclipse.lsp4j.services.LanguageClient;
import org.eclipse.lsp4j.PublishDiagnosticsParams;
import org.eclipse.xtext.ide.server.Document;
import org.eclipse.xtext.ide.server.UriExtensions;
import org.eclipse.xtext.resource.XtextResource;
import org.eclipse.lsp4j.services.LanguageServer;
import org.eclipse.lsp4j.Diagnostic;

import java.nio.charset.StandardCharsets;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.io.File;
import java.net.URLEncoder;

/**
 * Checks files that are currently being edited and
 * publishes diagnostics to the LanguageClient.
 */
public class DocumentRegistry {

    private static final Logger LOG = Logger.getLogger(LanguageServer.class);

    // FIXME: This is not right. UriExtensions should be a singleton.
    private static final UriExtensions uriExtensions = new UriExtensions();

    /** The DocumentRegistry singleton instance. */
    private static DocumentRegistry instance;
    /**
     * The LanguageClient instance through which to
     * communicate with the language client (e.g., by
     * sending diagnostics)
     */
    private LanguageClient client;
    /**
     * The location where any hidden files created by
     * `LFDocument`s should be placed
     */
    private final File workingDirectory = new File(".", ".lf-lsp");
    /**
     * The Lingua Franca documents tracked in the current
     * session
     */
    private final Map<LFDocument.ID, LFDocument> registry;

    /**
     * Returns the DocumentRegistry instance.
     * @return the DocumentRegistry instance
     */
    public static DocumentRegistry getInstance() {
        if (instance == null) {
            instance = new DocumentRegistry();
        }
        return instance;
    }

    /** Initializes the DocumentRegistry. */
    private DocumentRegistry() {
        registry = new HashMap<>();
    }

    /**
     * Sets the LanguageClient to which diagnostics should
     * be published.
     */
    public void setClient(LanguageClient client) {
        LOG.debug("Client is set as " + client);
        this.client = client;
    }

    /**
     * Updates information regarding the document specified
     * by document and resource.
     * @param document the Document object that provides the
     *                 content of the document
     * @param resource the object that provides metadata
     *                 about the document
     */
    public void refreshDocument(XtextResource resource, Document document) {
        final LFDocument.ID id = new LFDocument.ID(resource, document);
        LOG.debug("Refreshing document with target " + id.getTarget());
        if (!registry.containsKey(id)) {
            registry.put(id, new LFDocument(resource, document));
        }
        registry.get(id).refresh(document);
    }

    /**
     * Returns a location in the working directory in which
     * to save the file that is uniquely identified by
     * <code>f</code>. No guarantee is made about the
     * relationship between the returned file and
     * <code>f</code> except that for all <code>File</code>s
     * <code>f1</code>, <code>f2</code>, we have that <code>
     * getSaveLocation(uri1).equals(getSaveLocation(uri2))
     * </code> iff <code>uri1.equals(uri2)</code>.
     * @param f a File object that identifies a file
     * @return a File object corresponding to <code>f</code>
     */
    public File getSaveLocation(File f) { // FIXME: Remove. It is preferred to use the user-visible src directory for generated files when possible
        return new File(
            workingDirectory,
            URLEncoder.encode(f.getAbsolutePath(), StandardCharsets.UTF_8)
        );
    }

    /**
     * Sends the given diagnostics to the client.
     * @param resource the resource to which the diagnostics
     *                 correspond
     * @param diagnostics the diagnostics associated with
     *                    <code>resource</code>
     */
    public void publishDiagnostics(XtextResource resource, List<Diagnostic> diagnostics) {
        final PublishDiagnosticsParams publishDiagnosticsParams = new PublishDiagnosticsParams();
        publishDiagnosticsParams.setUri(uriExtensions.toUriString(resource.getURI()));
        publishDiagnosticsParams.setDiagnostics(diagnostics);
        client.publishDiagnostics(publishDiagnosticsParams);
    }
}