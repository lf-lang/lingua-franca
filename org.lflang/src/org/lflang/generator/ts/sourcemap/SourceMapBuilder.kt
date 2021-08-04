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
     * Returns the contents of a valid source map, according to the version 3
     * source map specification.
     * @param segments The SourceMapSegment containing all mappings
     * @return The contents of a valid source map, according to the version 3
     * source map specification
     */
    fun getSourceMap(segments: SourceMapSegment): String {
        return """
            |{
	    	|    "version": 3,
	    	|    "file": "${file.name}",
	    	|    "sources": ${getSourcesList()},
	    	|    "mappings": "${segments?.getMappings()}"
	        |}
        """.trimMargin()
    }

    /**
     * Returns a string representation of the "sources" property of the source
     * map.
     * @return a string representation of the "sources" property of the source
     * map
     */
    private fun getSourcesList(): String {
        val joiner = StringJoiner(", ")
        sources.forEach { joiner.add("\"${it.relativeTo(file.parentFile)}\"") } // FIXME: take source root into account? And why did it do like what we are seeing right now????
        return "[${joiner}]"
    }
}
