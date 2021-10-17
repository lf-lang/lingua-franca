package org.lflang.generator

import org.eclipse.emf.ecore.EObject
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.lflang.*
import org.lflang.JavaAstUtils.getInferredType
import org.lflang.lf.Time
import org.lflang.lf.TimeUnit
import org.lflang.lf.Type
import org.lflang.lf.Value


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

