package org.lflang.generator

import org.eclipse.emf.ecore.EObject
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.lflang.*
import org.lflang.lf.*


fun TargetTypes.getTargetInitializer(sv: StateVar): TargetCode =
    this.getTargetInitializer(sv.init, sv.type)

fun TargetTypes.getTargetInitializer(sv: Parameter): TargetCode =
    this.getTargetInitializer(sv.init, sv.type)

/**
 * Returns the target code for the actual value of a parameter
 * for the given instantiation. The value is defaulted to the
 * default value for the parameter if there is no explicit
 * assignment.
 */
fun TargetTypes.getTargetInitializer(param: Parameter, inst: Instantiation): TargetCode {
    val init = inst.parameters.firstOrNull { it.lhs == param }?.rhs
        ?: param.init
        ?: throw InvalidLfSourceException(inst, "No value for parameter ${param.name}")

    return getTargetInitializer(init, param.inferredType)
}


fun TargetTypes.getTargetTimeExpr(v: Value): TargetCode =
    this.getTargetExpr(v, InferredType.time())

/** If this is null, returns the literal 0. */
fun Value?.orZero(): Value =
    this ?: LfFactory.eINSTANCE.createLiteral().apply { literal = "0" }

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

