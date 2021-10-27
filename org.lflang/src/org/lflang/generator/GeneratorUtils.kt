package org.lflang.generator

import org.eclipse.emf.ecore.EObject
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.lflang.*
import org.lflang.lf.*


fun TargetTypes.getTargetInitializer(sv: StateVar): TargetCode =
    this.getTargetInitializer(sv.init, sv.type)

fun TargetTypes.getTargetInitializer(sv: Parameter): TargetCode =
    this.getTargetInitializer(sv.init, sv.type)

fun TargetTypes.getTargetTimeExpr(v: Value): TargetCode =
    this.getTargetExpr(v, InferredType.time())

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

