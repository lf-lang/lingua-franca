package org.lflang.generator.ts.sourcemap

import kotlinx.serialization.*

/**
 * Represents a source map segment that follows the specification provided in
 * the Source Map Revision 3 Proposal. Only supports segments that encode the
 * first four fields (column, source file, source line source column), and not
 * the fifth field (name).
 *
 * By their nature, source map segments must contain references to the segments
 * that precede them in order to be correctly as strings; for this reason, they
 * may be considered linked lists.
 *
 * @see <a href="https://sourcemaps.info/spec.html">Source Map Revision
 *     3 Proposal</a>
 * @param targetLine the absolute line in the target code corresponding to this
 * segment
 * @param targetColumn the absolute column in the target code corresponding to
 * this segment
 * @param sourceFile the absolute zero-based index of the source file, as listed
 * in the source map
 * @param sourceLine the absolute zero-based starting line in the original
 * source
 * @param sourceColumn the absolute zero-based starting column in the original
 * source
 */
@Serializable(with = SourceMapSegmentSerializer::class)
class SourceMapSegment(
    private val targetLine: Int,
    private val targetColumn: Int,
    private val sourceFile: Int,
    private val sourceLine: Int,
    private val sourceColumn: Int,
    private var precedingSegment: SourceMapSegment?
): Iterable<SourceMapSegment?> {

    /**
     * Iterates over this `SourceMapSegment` and those that
     * precede it in reverse order.
     */
    private class SourceMapSegmentIterator(var current: SourceMapSegment?): Iterator<SourceMapSegment?> {
        override fun hasNext(): Boolean {
            return current !== null
        }

        override fun next(): SourceMapSegment? {
            val ret = current
            current = current?.precedingSegment
            return ret
        }
    }

    fun getTargetLine(): Int = targetLine
    fun getTargetColumn(): Int = targetColumn
    fun getSourceFile(): Int = sourceFile
    fun getSourceLine(): Int = sourceLine
    fun getSourceColumn(): Int = sourceColumn

    /**
     * Generates a source map segment as specified in the Source Map Revision
     * 3 Proposal.
     */
    override fun toString(): String {
        val out = StringBuilder()
        if (precedingSegment !== null && precedingSegment!!.targetLine == targetLine) {
            // Preceded by nullity check, and the var doesn't mutate here.
            out.append(VLQ.toVLQ(targetColumn - precedingSegment!!.targetColumn))
        } else {
            out.append(VLQ.toVLQ(targetColumn))
        }
        if (precedingSegment !== null) {
            // Preceded by nullity check, and the var doesn't mutate here.
            out.append(VLQ.toVLQ(sourceFile - precedingSegment!!.sourceFile))
            out.append(VLQ.toVLQ(sourceLine - precedingSegment!!.sourceLine))
            out.append(VLQ.toVLQ(sourceColumn - precedingSegment!!.sourceColumn))
        } else {
            out.append(VLQ.toVLQ(sourceFile))
            out.append(VLQ.toVLQ(sourceLine))
            out.append(VLQ.toVLQ(sourceColumn))
        }
        return out.toString()
    }

    /**
     * Returns a string representation of all segments up to and including
     * the current one.
     */
    fun getMappings(): String {
        return getMappingsRecursive().toString()
    }

    /**
     * Returns a copy of this SourceMapSegment chain with all target lines and
     * columns shifted by lineDelta and columnDelta, respectively. This is a
     * non-mutative operation.
     * @param lineDelta The amount by which to shift the line numbers
     * @param columnDelta The amount by which to shift the column numbers
     */
    fun shifted(lineDelta: Int, columnDelta: Int): SourceMapSegment {
        return SourceMapSegment(
            targetLine = targetLine + lineDelta,
            targetColumn = targetColumn + columnDelta,
            sourceFile = sourceFile,
            sourceLine = sourceLine,
            sourceColumn = sourceColumn,
            precedingSegment = precedingSegment?.shifted(lineDelta, columnDelta)
        )
    }

    /**
     * Prepends a SourceMapSegment to the sequence of SourceMapSegments tracked
     * by this SourceMapSegment.
     * @param newTail the SourceMapSegment to be prepended
     */
    fun setTail(newTail: SourceMapSegment?) {
        var tail: SourceMapSegment = this
        while (tail.precedingSegment !== null) {
            tail = tail.precedingSegment!!
        }
        tail.precedingSegment = newTail
    }

    /**
     * Returns a StringBuilder representing all segments up to and including
     * the current one.
     */
    private fun getMappingsRecursive(): StringBuilder {
        if (precedingSegment === null) {
            return StringBuilder(";".repeat(targetLine)).append(toString())
        }
        // Preceded by nullity check, and the var doesn't mutate in this method.
        val builder = precedingSegment!!.getMappingsRecursive()
        if (targetLine == precedingSegment!!.targetLine) {
            builder.append(',')
        }
        builder.append(";".repeat(targetLine - precedingSegment!!.targetLine))
        builder.append(toString())
        return builder
    }

    companion object {

        /**
         * The characters that are permitted to be used in representations of
         * source map segments. May not be in any particular order.
         */
        const val CHARSET = VLQ.ORDERED_CHARSET + ",;"

        /**
         * Initializes a SourceMapSegment relative to precedingSegment using data
         * encoded in s, a string that complies with the Source Map Revision 3
         * Proposal.
         *
         * @param precedingSegment the SourceMapSegment preceding this
         *     SourceMapSegment
         * @param s a string that complies with the Source Map Revision 3
         *     Proposal
         * @param lineDelta the difference in line
         *     positions between the precedingSegment and
         *     this segment
         */
        fun fromString(
                precedingSegment: SourceMapSegment?,
                s: String,
                lineDelta: Int
        ): SourceMapSegment {
            val decoded = VLQ.fromVLQ(s)
            var targetLine = 0
            var targetColumn = decoded[0]
            var sourceFile = decoded[1]
            var sourceLine = decoded[2]
            var sourceColumn = decoded[3]
            if (precedingSegment !== null) {
                if (lineDelta == 0) {
                    targetColumn += precedingSegment.targetColumn
                }
                sourceFile += precedingSegment.sourceFile
                sourceLine += precedingSegment.sourceLine
                sourceColumn += precedingSegment.sourceColumn
                targetLine = precedingSegment.targetLine
            }
            targetLine += lineDelta
            return SourceMapSegment(
                    targetLine,
                    targetColumn,
                    sourceFile,
                    sourceLine,
                    sourceColumn,
                    precedingSegment
            )
        }
    }

    /**
     * Returns an iterator over this source map segment and
     * those that precede it in reverse order.
     */
    override fun iterator(): Iterator<SourceMapSegment?> {
        // Iterating backwards avoids the overhead of recursion and is probably a constant factor faster than
        //  iterating forwards. It is a little counterintuitive, but performance matters because this will
        //  probably be a very frequently performed operation, and because source maps can be extremely long.
        return SourceMapSegmentIterator(this)
    }
}