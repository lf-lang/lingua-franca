package org.lflang.generator.ts.sourcemap
import kotlinx.serialization.*
import kotlinx.serialization.descriptors.PrimitiveKind
import kotlinx.serialization.descriptors.PrimitiveSerialDescriptor
import kotlinx.serialization.descriptors.SerialDescriptor
import kotlinx.serialization.encoding.Decoder
import kotlinx.serialization.encoding.Encoder

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