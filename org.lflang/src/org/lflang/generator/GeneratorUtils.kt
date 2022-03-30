package org.lflang.generator

import org.eclipse.emf.ecore.EObject
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.lflang.ErrorReporter
import org.lflang.toPath
import org.lflang.toUnixString
import org.lflang.toTextTokenBased
import org.lflang.lf.Instantiation

/** A transparent type alias to document when a string contains target code. */
typealias TargetCode = String

/** Info about the location of an LF node. */
data class LocationInfo(
    val line: Int,
    val column: Int,
    val endLine: Int,
    val endColumn: Int,
    val fileName: String,
    val lfText: String
) {

    fun display() = "$fileName:$line"

    companion object {
        val MISSING = LocationInfo(1, 1, 1, 1, fileName = "<missing file>", lfText = "<missing text>")
    }
}


fun EObject.locationInfo(): LocationInfo {
    val node = NodeModelUtils.getNode(this)
    val start = NodeModelUtils.getLineAndColumn(node, node.offset)
    val end = NodeModelUtils.getLineAndColumn(node, node.endOffset)
    return LocationInfo(
        line = start.line,
        column = start.column,
        endLine = end.line,
        endColumn = end.column,
        fileName = this.eResource().toPath().toUnixString(),
        lfText = toTextTokenBased() ?: ""
    )
}
