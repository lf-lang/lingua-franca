package org.lflang.ide.document

import org.lflang.ide.DocumentRegistry
import org.lflang.generator.Main

import org.apache.log4j.Logger

import org.eclipse.lsp4j.services.LanguageServer
import org.eclipse.lsp4j.Position
import org.eclipse.lsp4j.Diagnostic
//import org.eclipse.xtext.ide.editor.quickfix.DiagnosticResolution
import org.eclipse.xtext.resource.XtextResource
import org.eclipse.xtext.ide.server.Document

import java.util.Map
import java.util.NavigableMap
import java.util.HashMap
import java.util.TreeMap
import java.util.List
import java.nio.file.Files
import java.io.File
import java.util.regex.Pattern
import java.util.regex.Matcher
import java.io.PrintWriter
import java.util.Map.Entry
import java.util.Comparator

/**
 * This class represents a Lingua Franca document and encapsulates the results
 * of any analyses performed on it.
 */
abstract class LFDocument {
	
	static val LOG = Logger.getLogger(LanguageServer)
	
	/**
	 * Represents a unique identifier for an LFDocument according to its
	 * absolute file system location AND its target language.
	 */
	static class ID {
		
		val static Pattern TARGET_PATTERN = Pattern.compile(
			'(?<=\\btarget\\s+)\\b(C|Cpp|Python|TypeScript)\\b'
		)
		
		val File file
		val TargetLanguage target
		
		/**
		 * Creates a unique identifier corresponding to the given document.
		 * @param resource the XtextResource that determines the document's
		 *     location in the file system
		 * @param document the Document that determines the document's target
		 *     language
		 */
		new(XtextResource resource, Document document) {
			// FIXME: Bad space complexity. should use an iterator.
			this(resource, getLineArray(document))
		}
		
		/**
		 * Creates a unique identifier corresponding to the given document.
		 * @param resource the XtextResource that determines the document's
		 *     location in the file system
		 * @param contents the lines of the Document that determine the
		 *     document's target language
		 */
		new(XtextResource resource, String[] contents) {
			// FIXME: This is very flimsy. What if someone writes "target C"
		    // before the real target declaration and then comments it out?
			file = (resource.getURI.isFile ?
			    new File(resource.getURI.toFileString) : null)
			var TargetLanguage result = null
			var i = 0
			var Matcher matcher
			while (result === null && i < contents.length) {
				matcher = TARGET_PATTERN.matcher(contents.get(i))
				if (matcher.find) {
					result = TargetLanguage.fromString(matcher.group)
				}
				i++
			}
			target = result
		}
		
		/**
		 * Returns whether this LFDocument.ID is equal to other. This is true
		 * iff other is an LFDocument.ID corresponding to the same file system
		 * location and the same target language, even if the original
		 * representation of the file system location is different.
		 * @param other the object against which this ID is to be compared
		 * @return whether this ID is equal to other
		 */
		override boolean equals(Object other) {
			if (!(other instanceof LFDocument.ID)) {
				return false
			}
			return ((other as LFDocument.ID).target == target
			    && (other as LFDocument.ID).file.equals(file)) 
		}
		
		/**
		 * Returns the target language of the LF file identified by this ID.
		 */
		def TargetLanguage getTarget() {
			return target
		}
	}
	
	static class PositionComparator implements Comparator<Position> {
		
		override compare(Position p1, Position p2) {
			if (p1.getLine != p2.getLine) {
				return p1.getLine - p2.getLine
			}
			return p1.getCharacter - p2.getCharacter
		}
		
	}
	
	val XtextResource resource
	var String[] lfLines
	// FIXME: Find a way to make targetLines and sourceMap a private member of
	// either LFDocument or the implementing class? Currently they are not
	// private because the quick update feature needs to permit this class to
	// write to these variables, but the thorough update feature needs to
	// permit the implementing classes to write to them. Because the thorough
	// update feature is more complex, one might argue that the coupling
	// between these variables and the implementing classes is tighter;
	// however, there is some common logic for processing the source map that
	// ought to be implemented here to avoid repetition.
	var protected String[] targetLines
	val protected NavigableMap<Position, Position> sourceMap
	//val Map<Diagnostic, DiagnosticResolution> diagnosticResolutions
	val ID id
		
	/**
	 * Initializes an LFDocument corresponding to resource with the contents
	 * of document.
	 * @param resource the LF file whose contents and semantics are modeled
	 *     by this LFDocument
	 * @param document the document whose contents and semantics are modeled
	 *     by this LFDocument
	 */
	new(XtextResource resource, Document document) {
		this(resource, getLineArray(document))
	}
	
	/**
	 * Initializes an LFDocument whose lines are contents corresponding to
	 * resource.
	 * @param resource the LF file whose contents and semantics are modeled
	 *     by this LFDocument
	 * @param contents the lines of code in the LF file whose contents and
	 *     semantics are modeled by this LFDocument
	 */
	new(XtextResource resource, String[] contents) {
		this.resource = resource
		this.lfLines = contents
		sourceMap = new TreeMap<Position, Position>(new PositionComparator)
		//diagnosticResolutions = new HashMap<Diagnostic, DiagnosticResolution>
		id = new ID(resource, contents)
	}
	
	/**
	 * Updates the model of the document represented by this LFDocument.
	 * Updates the client with any diagnostics associated with this document.
	 */
	def void refresh(Document document) {
		if (!attemptQuickUpdate(document)) {
			LOG.debug('Quick update failed. Attempting complete update...')
			// FIXME: Redundant work with attemptQuickUpdate?
			lfLines = getLineArray(document)
			updateTargetLinesAndSourceMap
		}
		val foundDiagnostics = diagnostics
		LOG.debug('Found ' + diagnostics.size + ' diagnostics: '
			+ foundDiagnostics.toString
		) // TODO remove
		DocumentRegistry.getInstance().publishDiagnostics(
			resource, foundDiagnostics
		)
	}
	
	/**
	 * Updates lfLines and targetLines to match the LF source code. Returns
	 * true iff changes to the document could be resolved without ambiguity.
	 * Otherwise, does nothing and returns false.
	 */
	def private boolean attemptQuickUpdate(Document document) {
		return false
	}
	
	/**
	 * Extracts an array of lines from a Document.
	 * @param document a document containing the desired content
	 * @return an array containing all lines in the document
	 */
	def private static String[] getLineArray(Document document) {
		val String[] contents = newArrayOfSize(document.getLineCount)
		// FIXME: Quadratic time!!! because the implementation of
		// Document.getLineContent(i) is linear wrt i. This is the
		// unacceptable cost of staying DRY.
		for (var i = 0; i < contents.length; i++) {
			contents.set(i, document.getLineContent(i))
		}
		return contents
	}
	
	/**
	 * Returns the Lingua Franca file location used for compilation.
	 */
	def private File getSrcFile() {
		val file = new File(
			new File(getCompileDir, 'src'), id.file.getName
		)
		file.parentFile.mkdirs
		return file
	}
	
	/**
	 * Returns the target language file produced by compilation.
	 */
	def private File getOutFile() {
		val file = new File(
			new File(
				new File(getCompileDir, 'src-gen'),
				id.file.getName.substring(0, id.file.getName.length - 3) //TODO make less ugly
			),
			id.file.getName.replaceAll('\\.lf\\Z', '.' + id.target.getExtension)
		)
		file.parentFile.mkdirs
		return file
	}
	
	/**
	 * Returns the working directory for any temporary files needed for
	 * compilation.
	 */
	def protected File getCompileDir() {
		// FIXME: I am not sure whether this ought to be cached rather than
		// re-computed each time. Caching is more complex because it creates
		// more state to keep track of, but it also feels silly to re-compute
		// this.
		val compileDir = DocumentRegistry.getInstance.getSaveLocation(
			new File(
				id.file.getParentFile,
				id.file.getName.replace('.', '_dot_')
			)
		)
		compileDir.mkdirs
		LOG.debug("CompileDir: " + compileDir.getAbsolutePath)
		return compileDir
	}
	
	/**
	 * Returns the result of compiling an LF document to its target language.
	 * @return the result of compiling lfDocumentContent to its target language
	 */
	def protected String[] compile() {
		// FIXME: Everything about this method is wrong.
		val writer = new PrintWriter(srcFile)
		for (var i = 0; i < lfLines.length; i++) {
			writer.println(lfLines.get(i))
		}
		writer.close
		Main.main(#[
			'--no-compile',
			srcFile.getAbsolutePath
		])
		LOG.debug("File contents: " + Files.readAllLines(outFile.toPath))
		return Files.readAllLines(outFile.toPath)
	}
	
	/**
	 * Returns the content of the Lingua Franca file.
	 * @return the content of the Lingua Franca file
	 */
	def protected String[] getLfLines() {
		return lfLines
	}
	
	/**
	 * Returns the position in the source code corresponding to targetPosition.
	 * @param targetPosition a position in the generated target code
	 * @return the position in the source code corresponding to targetPosition
	 */
	def protected Position adjustPosition(Position targetPosition) {
		// TODO add a check to make sure the two lines match up.
		val Entry<Position, Position> nearest = sourceMap.floorEntry(
			targetPosition
		)
		return new Position(
			nearest.getValue.getLine + (
				targetPosition.getLine - nearest.getKey.getLine
			),
			targetPosition.getCharacter
		)
	}
	
	/**
	 * Returns the unique identifier corresponding to this LFDocument. 
	 * @return the unique identifier corresponding to this LFDocument
	 */
	def ID getId() {
		return id
	}
	
	/**
	 * Returns the target language of this LFDocument.
	 * @return the target language of this LFDocument
	 */
	def TargetLanguage getTarget() {
		return id.target
	}
	
	/**
	 * Returns the lines of target code, all concatenated into one String.
	 * @return the lines of target code, all concatenated into one String
	 */
	def String getCombinedTargetLines() {
		val StringBuilder builder = new StringBuilder
		for (String line : targetLines) {
			builder.append(line);
		}
		return builder.toString
	}
	
	/**
	 * Updates the content of the generated target language file, as well as
	 * that of the source map.
	 */
	def protected abstract void updateTargetLinesAndSourceMap()
	
	/**
	 * Returns a list of diagnostics based on the current contents of
	 * targetLines.
	 */
	def protected abstract List<Diagnostic> getDiagnostics()
}
