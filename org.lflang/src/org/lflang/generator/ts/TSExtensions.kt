package org.lflang.generator.ts

import org.lflang.ASTUtils
import org.lflang.TimeValue
import org.lflang.generator.getTargetTimeExpr
import org.lflang.isBank
import org.lflang.isMultiport
import org.lflang.lf.Action
import org.lflang.lf.Expression
import org.lflang.lf.Parameter
import org.lflang.lf.ParameterReference
import org.lflang.lf.Port
import org.lflang.lf.WidthSpec
import org.lflang.toText

/**
 * The following definition provide extension that are useful for TypeScript target.
 *
 *  @author {Hokeun Kim <hokeunkim@berkeley.edu>}
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

/**
 * Given a connection 'delay' predicate, return a string that represents the
 * time value in TypeScript code.
 */
fun getNetworkDelayLiteral(delay: Expression?): String {
    var additionalDelayString = "TimeValue.NEVER()"
    if (delay != null) {
        val tv: TimeValue
        tv = if (delay is ParameterReference) {
            // The delay is given as a parameter reference. Find its value.
            ASTUtils.getDefaultAsTimeValue(delay.parameter)
        } else {
            ASTUtils.getLiteralTimeValue(delay)
        }
        additionalDelayString = tv.toTsTime();
    }
    return additionalDelayString
}
fun Expression.toTsTime(): String = TSTypes.getTargetTimeExpr(this)
fun TimeValue.toTsTime(): String = TSTypes.getTargetTimeExpr(this)
