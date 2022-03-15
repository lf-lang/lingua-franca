package org.lflang.generator.ts

import org.lflang.federated.FederateInstance
import org.lflang.isBank
import org.lflang.isMultiport
import org.lflang.lf.Action
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
fun getActionType(action: Action, federate: FederateInstance): String {
    // Special handling for the networkMessage action created by
    // FedASTUtils.makeCommunication(), by assigning TypeScript
    // Buffer type for the action. Action<Buffer> is used as
    // FederatePortAction in federation.ts.
    if (action in federate.networkMessageActions) {
        return "Buffer"
    } else if (action.type != null) {
        return action.type.getTargetType()
    } else {
        return "Present"
    }
}
