package org.lflang.generator.ts

import org.lflang.lf.Parameter
import org.lflang.lf.Port
import org.lflang.lf.Type
import org.lflang.lf.WidthSpec
import org.lflang.toText

/**
 * The following definition provide extension that are useful for TypeScript target.
 *
 *  @author {Hokeun Kim <hokeunkim@berkeley.edu>}
 */
fun WidthSpec.toTSCode(): String = terms.joinToString(" + ") {
    when {
        it.parameter != null -> "${it.parameter.name}"
        it.code != null -> it.code.toText()
        else -> it.width.toString()
    }
}

private fun Type.getTargetType(): String = TSTypes.getTargetType(this)

/**
 * Return a TS type for the specified port.
 * If the type has not been specified, return
 * "Present" which is the base type for ports.
 * @param port The port
 * @return The TS type.
 */
fun getPortType(port: Port): String {
    if (port.type != null) {
        return port.type.getTargetType()
    } else {
        return "Present"
    }
}

fun Parameter.getTargetType(): String = TSTypes.getTargetType(this)
