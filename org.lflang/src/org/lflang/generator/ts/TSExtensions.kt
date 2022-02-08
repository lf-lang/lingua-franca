package org.lflang.generator.ts

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