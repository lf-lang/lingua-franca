package org.lflang.ide

import org.lflang.ide.document.LFDocument
import org.lflang.ide.document.LFWithCCppTarget
import org.lflang.ide.document.TargetLanguage

import org.apache.log4j.Logger

import org.eclipse.lsp4j.services.LanguageClient
import org.eclipse.lsp4j.PublishDiagnosticsParams
import org.eclipse.xtext.ide.server.Document
import org.eclipse.xtext.ide.server.UriExtensions
import org.eclipse.xtext.resource.XtextResource
import org.eclipse.lsp4j.services.LanguageServer
import org.eclipse.lsp4j.Diagnostic
import org.eclipse.lsp4j.DiagnosticSeverity
import org.eclipse.lsp4j.Range
import org.eclipse.lsp4j.Position

import java.util.List
import java.util.ArrayList
import java.util.Map
import java.util.HashMap
import java.io.File
import java.net.URLEncoder

/**
 * Checks files that are currently being edited and publishes diagnostics
 * to the LanguageClient.
 */
class DocumentRegistry {
	
	static val LOG = Logger.getLogger(LanguageServer)
	
	// FIXME This is not right. UriExtensions should be a singleton.
	static val uriExtensions = new UriExtensions()
	
	/** The DocumentRegistry instance. (This class is a singleton.) */
	static DocumentRegistry instance
	/**
	 * The LanguageClient instance through which to communicate with the
	 * language client (e.g., by sending diagnostics)
	 */
	var LanguageClient client
	/**
	 * The location where any hidden files created by `LFDocument`s should be
	 * placed
	 */
	val File workingDirectory = new File('.', '.lf-lsp')
	/** The Lingua Franca documents tracked in the current session */
	val Map<LFDocument.ID, LFDocument> registry
	
	/**
	 * Returns the DocumentRegistry instance.
	 * @return the DocumentRegistry instance
	 */
	def static getInstance() {
		if (instance === null) {
			instance = new DocumentRegistry
		}
		return instance
	}
	
	/** Initializes the DocumentRegistry. */
	private new() {
		registry = new HashMap
	}
	
	/**
	 * Sets the LanguageClient to which diagnostics should be published.
	 */
	def setClient(LanguageClient client) {
		LOG.debug('Client is set as ' + client.toString)
		this.client = client
	}
	
	/**
	 * Updates information regarding the document specified by document and
	 * resource.
	 * @param document the Document object that provides the content of the
	 *     document
	 * @param resource the object that provides metadata about the document
	 */
	def refreshDocument(XtextResource resource, Document document) {
		val id = new LFDocument.ID(resource, document)
		if (!registry.containsKey(id)) {
			registry.put(
				id,
				switch (id.target) {
					case TargetLanguage.C: new LFWithCCppTarget(resource, document)
					case TargetLanguage.CPP: new LFWithCCppTarget(resource, document)
					// TODO Add support for other target languages
					default: new LFWithCCppTarget(resource, document)
				}
			)
		}
		registry.get(id).refresh(document)
	}
	
	/**
	 * Returns a location in the working directory in which to save the file
	 * that is uniquely identified by uri. No guarantee is made about the
	 * relationship between the returned file and uri except that for all
	 * URIs uri1, uri2, we have that
	 * getSaveLocation(uri1).equals(getSaveLocation(uri2)) if and only if
	 * uri1.equals(uri2).
	 * @param f a File object that identifies a file
	 * @return a File object corresponding to uri
	 */
	def File getSaveLocation(File f) {
		return new File(
			workingDirectory,
			URLEncoder.encode(f.getAbsolutePath, "UTF-8")
		)
	}
	
	/**
	 * Sends the given diagnostics to the client.
	 * @param resource the resource to which the diagnostics correspond
	 * @param diagnostics the diagnostics associated with resource
	 */
	def void publishDiagnostics(XtextResource resource, List<Diagnostic> diagnostics) {
		val publishDiagnosticsParams = new PublishDiagnosticsParams
		publishDiagnosticsParams.setUri(uriExtensions.toUriString(resource.getURI))
		publishDiagnosticsParams.setDiagnostics(diagnostics)
		client.publishDiagnostics(publishDiagnosticsParams)
	}
}