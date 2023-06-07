package org.lflang.generator

import org.eclipse.emf.ecore.EObject
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.lflang.InferredType
import org.lflang.inferredType
import org.lflang.lf.Expression
import org.lflang.lf.Initializer
import org.lflang.lf.Instantiation
import org.lflang.lf.LfFactory
import org.lflang.lf.Parameter
import org.lflang.lf.StateVar
import org.lflang.toPath
import org.lflang.toTextTokenBased
import org.lflang.toUnixString

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


/**
 * Returns the target code for the initial value of [sv].
 */
fun TargetTypes.getTargetInitializer(sv: StateVar): TargetCode =
    this.getTargetInitializer(sv.init, sv.type)

/**
 * Returns the target code for the default value of the [param].
 */
fun TargetTypes.getTargetInitializer(param: Parameter): TargetCode =
    this.getTargetInitializer(param.init, param.type)

/**
 * Returns the target code for the [getActualValue] of the
 * param for this instantiation.
 */
fun TargetTypes.getTargetInitializer(param: Parameter, inst: Instantiation): TargetCode {
    val init = inst.getActualValue(param)
    return getTargetInitializer(init, param.type)
}

/**
 * Return the actual value of a parameter for the given instantiation.
 * The value is defaulted to the default value for the parameter if
 * there is no explicit assignment. If there is no default value, the
 * source code is invalid (param is required)
 */
fun Instantiation.getActualValue(param: Parameter): Initializer =
    parameters.firstOrNull { it.lhs == param }?.rhs
        ?: param.init
        ?: throw InvalidLfSourceException(this, "No value for parameter ${param.name}")


/**
 * Return the target code for the given expression, given
 * that it's a time expression.
 */
fun TargetTypes.getTargetTimeExpr(v: Expression): TargetCode =
    this.getTargetExpr(v, InferredType.time())

/** If this is null, return the literal 0. */
fun Expression?.orZero(): Expression =
    this ?: LfFactory.eINSTANCE.createLiteral().apply { literal = "0" }

