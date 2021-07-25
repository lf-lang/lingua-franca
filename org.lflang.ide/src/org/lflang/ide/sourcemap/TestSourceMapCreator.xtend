package org.lflang.ide.sourcemap

import org.eclipse.xtext.ide.server.Document
import java.io.PrintWriter
import java.nio.file.Files
import java.nio.file.FileSystems

/**
 * Creates a source map matching positions in a generated document to
 * positions in a source document. This is strictly intended for 
 * qualitative, ad-hoc testing of other code that is related to source maps.
 * No strong guarantees are made about the correctness of this class's
 * behavior.
 */
class TestSourceMapCreator {
	
	/**
	 * Generates a mock sourceMap associating positions in the generated file
	 * with positions in the source file.
	 * 
	 * @param generatedFile the name of a file containing generated target code
	 * @param generated a representation of a file containing generated target
	 *     code
	 * @param sourceFile the name of a corresponding source file
	 * @param source a representation of the corresponding source file
	 * @return a mock sourceMap associating positions in the generated file
	 *     with positions in the source file
	 */
	def static String sourceMap(
		String generatedFile, Document generated,
		String sourceFile, Document source
	) '''
	    {
	    	"version": 3,
	    	"file": "«generatedFile»",
	    	"sources": ["«sourceFile»"],
	    	"mappings": "«mappings(generated, source)»"
	    }
	'''
	
	/**
	 * Generates the mappings field of a source map file.
	 * 
	 * @param generated the generated document
	 * @param source the source document
	 * @return the mappings field of a source map file
	 */
	def private static String mappings(Document generated, Document source) {
		var SourceMapSegment segment = null
		for (var line = 0; line < generated.getLineCount; line++) {
			var generatedLine = generated.getLineContent(line)
			for (var sourceLine = 0; sourceLine < source.getLineCount; sourceLine++) {
				if (source.getLineContent(sourceLine) == generatedLine) {
					var i = 0
					while (i != -1) {
						segment = new SourceMapSegment(line, i, segment, sourceLine, i, 0)
						i = generatedLine.indexOf(' ', i + 1) + 1
						if (i === 0) i--
					}
					segment = new SourceMapSegment(
						line, generatedLine.length, segment, sourceLine,
						generatedLine.length, 0
					)
				}
			}
		}
		return segment.getMappings
	}
	
	def static void main(String[] args) {
		var PrintWriter out = new PrintWriter(FileSystems.getDefault().getPath(
			'..', 'vscode-extension', 'out', 'highlight.js.map'
		).toFile)
		out.print(sourceMap(
			'highlight.js',
			new Document(0, Files.readString(FileSystems.getDefault().getPath(
				'..', 'vscode-extension', 'out', 'highlight.js'
			))),
			'../src/highlight.ts',
			new Document(0, Files.readString(FileSystems.getDefault().getPath(
				'..', 'vscode-extension', 'src', 'highlight.ts'
			)))
		))
		out.close
		out = new PrintWriter(FileSystems.getDefault().getPath(
			'..', 'vscode-extension', 'out', 'extension.js.map'
		).toFile)
		out.print(sourceMap(
			'highlight.js',
			new Document(0, Files.readString(FileSystems.getDefault().getPath(
				'..', 'vscode-extension', 'out', 'extension.js'
			))),
			'../src/highlight.ts',
			new Document(0, Files.readString(FileSystems.getDefault().getPath(
				'..', 'vscode-extension', 'src', 'extension.ts'
			)))
		))
		out.close
	}
}