package org.lflang.generator.ts

import org.lflang.TimeValue
import org.lflang.generator.getTargetTimeExpr
import org.lflang.isBank
import org.lflang.isMultiport
import org.lflang.lf.Action
import org.lflang.lf.Expression
import org.lflang.lf.Port
import org.lflang.lf.WidthSpec
import org.lflang.toText

/**
 * The following definition provide extension that are useful for TypeScript target.
 *
 *  @author Hokeun Kim
 */
fun WidthSpec.toTSCode(): String = terms.joinToString(" + ") {
    when {
        it.parameter != null -> it.parameter.name
        it.port != null -> with(it.port) {
            if (container?.isBank == true) {
                if ((variable as Port).isMultiport) "this.${container.name}.all().length * this.${container.name}.all()[0].${variable.name}.width()"
                else "this.${container.name}.all().length"
            } else {
                if ((variable as Port).isMultiport) "this.${container.name}.${variable.name}.width()"
                else "1"
            }
        }
        it.code != null -> it.code.toText()
        else -> it.width.toString()
    }
}

/**
 * Return a TS type for the specified port.
 * If the type has not been specified, return
 * "Present" which is the base type for ports.
 * @return The TS type.
 */
val Port.tsPortType: String
    get() = type?.let { TSTypes.getTargetType(it) } ?: "Present"

/**
 * Return a TS type for the specified action.
 * If the type has not been specified, return
 * "Present" which is the base type for Actions.
 * @return The TS type.
 */
val Action.tsActionType: String
    get() = type?.let { TSTypes.getTargetType(it) } ?: "Present"

fun Expression.toTsTime(): String = TSTypes.getTargetTimeExpr(this)
fun TimeValue.toTsTime(): String = TSTypes.getTargetTimeExpr(this)
