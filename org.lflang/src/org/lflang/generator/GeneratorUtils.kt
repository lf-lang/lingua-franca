package org.lflang.generator

import org.eclipse.emf.ecore.EObject
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.lflang.*
import org.lflang.lf.Time
import org.lflang.lf.TimeUnit
import org.lflang.lf.Type
import org.lflang.lf.Value


/** A transparent type alias to document when a string contains target code. */
typealias TargetCode = String

/**
 *  Returns a target type inferred from the type node, or the
 *  initializer list. If both are absent, then the undefined
 *  type is returned.
 */
fun TargetTypes.getTargetType(type: Type?, init: List<Value>? = null): TargetCode {
    return this.getTargetType(getInferredType(type, init))
}

fun getInferredType(type: Type?, init: List<Value>?): InferredType =
    type?.let(InferredType::fromAST)
        ?: init?.let(ASTUtils::getInferredType)
        ?: InferredType.undefined()

fun TargetTypes.getTargetInitializer(init: List<Value>, type: Type? = null, initWithBraces: Boolean = false): TargetCode {
    val inferredType = getInferredType(type, init)
    if (init.size == 1) {
        return getTargetExpr(init[0], inferredType)
    }
    val targetValues = init.map { getTargetExpr(it, inferredType) }
    return when {
        inferredType.isFixedSizeList    -> getFixedSizeListInitExpression(targetValues, initWithBraces)
        inferredType.isVariableSizeList -> getVariableSizeListInitExpression(targetValues, initWithBraces)

        else                            -> this.missingExpr
    }
}

fun TargetTypes.getTargetExpr(value: Value, inferred: InferredType? = null): TargetCode =
    with(value) {
        if (this.isZero && inferred?.isTime == true)
            getTargetTimeExpression(0, TimeUnit.NONE)
        else
            parameter?.name?.let(this@getTargetExpr::escapeIdentifier) // only escaped if using LF syntax
                ?: time?.let(this@getTargetExpr::getTargetTimeExpr)
                ?: literal // here we don't escape
                ?: code?.toText() // nore here
                ?: throw IllegalStateException("Invalid value $value")
    }

fun TargetTypes.getTargetTimeExpr(tv: TimeValue): TargetCode = getTargetTimeExpression(tv.time, tv.unit)
fun TargetTypes.getTargetTimeExpr(t: Time): TargetCode = getTargetTimeExpression(t.interval.toLong(), t.unit)


/** Info about the location of an LF node. */
data class LocationInfo(val line: Int, val fileName: String, val lfText: String) {

    fun display() = "$fileName:$line"

    companion object {
        val MISSING = LocationInfo(line = 1, fileName = "<missing file>", lfText = "<missing text>")
    }
}


fun EObject.locationInfo(): LocationInfo {
    val node = NodeModelUtils.getNode(this)
    return LocationInfo(
        line = node.startLine,
        fileName = this.eResource().toPath().toUnixString(),
        lfText = toTextTokenBased() ?: ""
    )
}

