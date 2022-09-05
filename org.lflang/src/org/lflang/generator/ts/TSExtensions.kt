package org.lflang.generator.ts

import org.lflang.ASTUtils
import org.lflang.TimeValue
import org.lflang.isBank
import org.lflang.isMultiport
import org.lflang.lf.Action
import org.lflang.lf.Expression
import org.lflang.lf.Parameter
import org.lflang.lf.ParameterReference
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

/**
 * Return a TS type for the specified action.
 * If the type has not been specified, return
 * "Present" which is the base type for Actions.
 * @param action The action
 * @return The TS type.
 */
fun getActionType(action: Action): String {
    if (action.type != null) {
        return action.type.getTargetType()
    } else {
        return "Present"
    }
}

fun timeInTargetLanguage(value: TimeValue): String {
    return if (value.unit != null) {
        "TimeValue.${value.unit.canonicalName}(${value.time})"
    } else {
        // The value must be zero.
        "TimeValue.zero()"
    }
}

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
        additionalDelayString = timeInTargetLanguage(tv)
    }
    return additionalDelayString
}