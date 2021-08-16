package org.lflang.generator.ts.sourcemap
import kotlinx.serialization.*
import kotlinx.serialization.descriptors.PrimitiveKind
import kotlinx.serialization.descriptors.PrimitiveSerialDescriptor
import kotlinx.serialization.descriptors.SerialDescriptor
import kotlinx.serialization.encoding.Decoder
import kotlinx.serialization.encoding.Encoder
import kotlinx.serialization.json.Json
import java.io.File
import java.io.IOException
import java.util.regex.Pattern
import kotlin.jvm.Throws

val SOURCE_MAPPING_URL_PATTERN: Pattern = Pattern.compile("//# sourceMappingURL=(?<url>.*)") // FIXME: Make more specific than .*?

@Serializable
data class SourceMap(val version: Int, val file: String, val sources: List<String>, val mappings: SourceMapSegment?)

object SourceMapSegmentSerializer : KSerializer<SourceMapSegment?> {
    override val descriptor: SerialDescriptor = PrimitiveSerialDescriptor("SourceMapSegment", PrimitiveKind.STRING)
    override fun serialize(encoder: Encoder, value: SourceMapSegment?) {
        encoder.encodeString(value?.getMappings() ?: "")
    }

    override fun deserialize(decoder: Decoder): SourceMapSegment? {
        var head: SourceMapSegment? = null
        var lineDelta = 0
        for (line: String in decoder.decodeString().split(';')) {
            for (segment: String in line.split(',')) {
                head = SourceMapSegment.fromString(head, segment, lineDelta)
                lineDelta = 0
            }
            lineDelta++
        }
        return head
    }

}

/**
 * Returns the source map object associated with the
 * generated file `f`, or `null` if there is no such object.
 * @param contents the contents of a generated file in the
 * target language that may point to a source map
 * @return the source map associated with `f`
 */
@Throws(IOException::class)
fun getSourceMapOf(contents: List<String>, directory: File): SourceMap? {
    val sourceMapFile = getSourceMappingFileOf(contents, directory)
    if (sourceMapFile === null) return null
    return Json.decodeFromString<SourceMap>(sourceMapFile.readText())
}

/**
 * Returns the source mapping file that is pointed to by a
 * comment at the end of the file `f`.
 * @param contents the file contents to be searched for a
 * source mapping URL
 * @param directory the parent directory of the file whose
 * contents are to be searched
 * @return the source mapping file that is pointed to by a
 * comment at the end of `f`
 */
private fun getSourceMappingFileOf(contents: List<String>, directory: File): File? {
    // FIXME: Searches the entire document for the source map. If it isn't there, and the file is long,
    //  then this might not be acceptable. Furthermore, it might be unnecessarily dangerous: The source map
    //  link should be at the bottom of the document. In principle, something else that matches the regex
    //  could exist elsewhere.
    contents.asReversed().forEach { // asReversed appears to be O(1) based on the source code
        val matcher = SOURCE_MAPPING_URL_PATTERN.matcher(it)
        if (matcher.find()) {
            return File(directory, matcher.group("url"))
        }
    }
    return null
}