package org.lflang.generator

import org.eclipse.emf.ecore.EObject
import org.eclipse.xtext.nodemodel.util.NodeModelUtils
import org.lflang.*
import org.lflang.lf.*


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
 * Check whether code can be generated; report any problems
 * and inform the context accordingly.
 * @return Whether it is possible to generate code.
 */
fun canGenerate(
    errorsOccurred: Boolean,
    mainDef: Instantiation?,
    errorReporter: ErrorReporter,
    context: LFGeneratorContext
): Boolean {
    // stop if there are any errors found in the program by doGenerate() in GeneratorBase
    if (errorsOccurred) {
        JavaGeneratorUtils.finish(context, GeneratorResult.FAILED)
        return false
    }
    // abort if there is no main reactor
    if (mainDef == null) {
        errorReporter.reportWarning("WARNING: The given Lingua Franca program does not define a main reactor. Therefore, no code was generated.")
        JavaGeneratorUtils.finish(context, GeneratorResult.NOTHING)
        return false
    }
    return true
}
