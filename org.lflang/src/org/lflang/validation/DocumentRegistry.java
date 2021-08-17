package org.lflang.validation;

import org.lflang.validation.document.LFDocument;

import org.eclipse.emf.ecore.EObject;

import java.nio.charset.StandardCharsets;
import java.util.Map;
import java.util.HashMap;
import java.io.File;
import java.net.URLEncoder;

/**
 * Checks files that are currently being edited and
 * publishes diagnostics to the LanguageClient.
 */
public class DocumentRegistry {

    /** The DocumentRegistry singleton instance. */
    private static DocumentRegistry instance;
    /**
     * The location where any hidden files created by
     * `LFDocument`s should be placed
     */
    private final File workingDirectory = new File(".", ".lf-lsp"); // FIXME: Make temporary
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
     * Gets the document of which <code>parseRoot</code> is
     * the parse root.
     * @param parseRoot the parse root of the LF document of
     *                  interest
     */
    public LFDocument getDocument(EObject parseRoot) {
        final LFDocument.ID id = new LFDocument.ID(parseRoot);
        if (!registry.containsKey(id)) {
            registry.put(id, new LFDocument(parseRoot));
        }
        return registry.get(id);
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
}