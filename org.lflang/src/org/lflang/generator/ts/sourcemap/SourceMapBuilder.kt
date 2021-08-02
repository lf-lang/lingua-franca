package org.lflang.generator.ts.sourcemap

import java.io.File
import java.util.*
import kotlin.collections.ArrayList

/**
 * Accumulates the data required to produce a JavaScript-style source map.
 * @see <a href="https://sourcemaps.info/spec.html">Source Map Revision
 * 3 Proposal</a>
 * @param file the name of the generated code that this source map is associated
 * with
 * @param sourceRoot the path that is prepended to the individual entries in the
 * “source” field
 */
class SourceMapBuilder(
    private val file: File,
    private val sourceRoot: File = File("")
) {
    /** The original sources used by the “mappings” entry */
    private val sources: MutableList<File> = ArrayList()
    /** The latest segment of mapping data that is stored in this
     * SourceMapBuilder */
    private var headSegment: SourceMapSegment? = null

    /**
     * Returns the numeric identifier that source map segments should use to
     * refer to the given file.
     * @param f the file that must be identified with a number
     * @return the numeric identifier that source map segments should use to
     * refer to the given file
     */
    fun sourceID(f: File): Int {
        // Warning: This implementation does not scale with an acceptable time
        // complexity wrt the length of sources. It is assumed that the
        // `sources` list will never contain more than a few entries. If this
        // becomes a problem, a map should be used instead.
        val idx = sources.indexOf(f)
        return if (idx >= 0) idx else {
            sources.add(f)
            sourceID(f)
        }
    }

    /**
     * Appends a segment (which may be the head of a linked list of segments) to
     * this SourceMapBuilder.
     * @param segment the segment or head of a segment chain that must be
     * appended
     */
    fun addSegment(segment: SourceMapSegment) {
        segment.setTail(headSegment)
        headSegment = segment
    }

    /**
     * Returns the contents of a valid source map, according to the version 3
     * source map specification.
     * @return the contents of a valid source map, according to the version 3
     * source map specification
     */
    fun getSourceMap(): String {
        return """
            |{
	    	|    "version": 3,
	    	|    "file": "${file.name}",
	    	|    "sources": ${getSourcesList()},
	    	|    "mappings": "${headSegment?.getMappings()}"
	        |}
        """.trimIndent()
    }

    /**
     * Returns a string representation of the "sources" property of the source
     * map.
     * @return a string representation of the "sources" property of the source
     * map
     */
    private fun getSourcesList(): String {
        val joiner = StringJoiner("\", \"")
        sources.forEach { joiner.add(it.resolve(file).toString()) }
        return "[${joiner}]"
    }
}
