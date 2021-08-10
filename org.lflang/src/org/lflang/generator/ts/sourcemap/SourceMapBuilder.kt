package org.lflang.generator.ts.sourcemap

import java.io.File
import kotlin.collections.ArrayList
import kotlinx.serialization.json.Json
import kotlinx.serialization.encodeToString

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
        // Warning: This implementation does not scale with an appropriate time
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
    fun getSourceMap(segments: SourceMapSegment?): String {
        return Json.encodeToString(SourceMap(
            version = 3,
            file = file.name,
            sources = sources.map {it.relativeTo(file.parentFile).toString()},
            mappings = segments
        ))
    }
}
