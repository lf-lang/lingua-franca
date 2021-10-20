package org.lflang.generator

import org.eclipse.emf.ecore.EObject
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.lflang.InferredType
import org.lflang.lf.Parameter
import org.lflang.lf.StateVar
import org.lflang.lf.Value
import org.lflang.toPath
import org.lflang.toTextTokenBased
import org.lflang.toUnixString


fun TargetTypes.getTargetInitializer(sv: StateVar): TargetCode =
    this.getTargetInitializer(sv.init, sv.type)

fun TargetTypes.getTargetInitializer(sv: Parameter): TargetCode =
    this.getTargetInitializer(sv.init, sv.type)

fun TargetTypes.getTargetTimeExpr(v: Value): TargetCode =
    this.getTargetExpr(v, InferredType.time())


/** A transparent type alias to document when a string contains target code. */
typealias TargetCode = String

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

