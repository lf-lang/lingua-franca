package org.lflang.ide.sourcemap

import org.eclipse.lsp4j.Position

/**
 * Represents a source map segment that follows the specification provided in
 * the Source Map Revision 3 Proposal. Only supports segments that encode the
 * first four fields (column, source file, source line source column), and not
 * the fifth field (name).
 * 
 * @see <a href="https://sourcemaps.info/spec.html">Source Map Revision
 *     3 Proposal</a>
 */
class SourceMapSegment {
	/** The absolute line in the target code corresponding to this segment. */
	var int targetLine
	/** The absolute column in the target code corresponding to this segment. */
	var int targetColumn
	/** The absolute zero-based index of the source file, as listed in the
	 * source map.
	 */
	var int sourceFile
	/** The absolute zero-based starting line in the original source. */
	var int sourceLine
	/** The absolute zero-based starting column in the original source. */
	var int sourceColumn
	/** The SourceMapSegment preceding this SourceMapSegment. Null if this is
	 * the first SourceMapSegment in the source map.
	 */
	var SourceMapSegment precedingSegment
	
	/**
	 * Initializes a SourceMapSegment.
	 * 
	 * @param targetPos the position in the generated target code that this segment
	 *     represents
	 * @param precedingSegment the SourceMapSegment preceding this SourceMapSegment
	 * @param sourceCurrent the source code position corresponding to targetPos
	 * @param currentSourceIdx the index of the source code document corresponding
	 *     to targetPos
	 */
	new(
		Position targetPos,
		SourceMapSegment precedingSegment,
		Position sourceCurrent,
		int currentSourceIdx
	) {
		this(
			targetPos.getLine, targetPos.getCharacter,
			precedingSegment,
			sourceCurrent.getLine, sourceCurrent.getCharacter,
			currentSourceIdx
		)
	}
	
	/**
	 * Initializes a SourceMapSegment.
	 * 
	 * @param targetLine the line in the generated target code that this
	 *     segment represents
	 * @param targetColumn the column in the generated target code that this
	 *     segment represents
	 * @param precedingSegment the SourceMapSegment preceding this
	 *     SourceMapSegment
	 * @param sourceLine the source code line corresponding to the target
	 *     position
	 * @param sourceColumn the source code column corresponding to the
	 *     target column
	 * @param currentSourceIdx the index of the source code document
	 *     corresponding to targetPos
	 */
	new(
		int targetLine, int targetColumn,
		SourceMapSegment precedingSegment,
		int sourceLine, int sourceColumn,
		int currentSourceIdx
	) {
		this.targetLine = targetLine
		this.targetColumn = targetColumn
		this.sourceLine = sourceLine
		this.sourceColumn = sourceColumn
		this.precedingSegment = precedingSegment
		this.sourceFile = currentSourceIdx
	}
	
	/**
	 * Initializes a SourceMapSegment relative to precedingSegment using data
	 * encoded in s, a string that complies with the Source Map Revision 3
	 * Proposal.
	 * 
	 * @param precedingSegment the SourceMapSegment preceding this
	 *     SourceMapSegment
	 * @param s a string that complies with the Source Map Revision 3
	 *     Proposal
	 * @param incrementLine whether the target line represented by s is the
	 *     is the line that follows that of precedingSegment, rather than the
	 *     same line
	 */
	new(SourceMapSegment precedingSegment, String s, boolean incrementLine) {
		val int[] decoded = VLQ.fromVLQ(s)
		this.targetColumn = decoded.get(0)
		this.sourceFile = decoded.get(1)
		this.sourceLine = decoded.get(2)
		this.sourceColumn = decoded.get(3)
		this.targetLine = 0
		if (precedingSegment !== null) {
			if (precedingSegment.targetLine == targetLine) {
				this.targetColumn += precedingSegment.targetColumn
				this.sourceFile += precedingSegment.sourceFile
				this.sourceLine += precedingSegment.sourceLine
				this.sourceColumn += precedingSegment.sourceColumn
				this.targetLine = precedingSegment.targetLine
			}
		}
		if (incrementLine) {
			targetLine++
		}
	}
	
	/**
	 * Generates a source map segment as specified in the Source Map Revision
	 * 3 Proposal.
	 */
	override String toString() {
		val out = new StringBuilder
		out.append(VLQ.toVLQ(targetColumn))
		if (precedingSegment !== null) {
			out.append(VLQ.toVLQ(sourceFile - precedingSegment.sourceFile))
			out.append(VLQ.toVLQ(sourceLine - precedingSegment.sourceLine))
			out.append(VLQ.toVLQ(sourceColumn - precedingSegment.sourceColumn))
		} else {
			out.append(VLQ.toVLQ(sourceFile))
			out.append(VLQ.toVLQ(sourceLine))
			out.append(VLQ.toVLQ(sourceColumn))
		}
		return out.toString
	}
	
	/**
	 * Returns a string representation of all segments up to and including
	 * the current one.
	 */
	def String getMappings() {
		getMappingsRecursive.toString
	}
	
	/**
	 * Returns a StringBuilder representing all segments up to and including
	 * the current one.
	 */
	def private StringBuilder getMappingsRecursive() {
		if (precedingSegment === null) {
			return new StringBuilder(toString)
		}
		val builder = precedingSegment.getMappingsRecursive
		if (targetLine == precedingSegment.targetLine) {
			builder.append(',')
		}
		builder.append(';'.repeat(targetLine - precedingSegment.targetLine))
		builder.append(toString)
		return builder
	}
}