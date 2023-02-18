package org.lflang.generator.ts

import org.lflang.ASTUtils
import org.lflang.Target
import org.lflang.generator.DelayBodyGenerator
import org.lflang.lf.Action
import org.lflang.lf.VarRef

object TSDelayBodyGenerator : DelayBodyGenerator {
    /**
     * Return a TS type for the specified action.
     * If the type has not been specified, return
     * "Present" which is the base type for Actions.
     * @param action The action
     * @return The TS type.
     */
    private fun getActionType(action: Action): String {
        return if (action.type != null) {
            TSTypes.getTargetType(action.type)
        } else {
            "Present"
        }
    }

    override fun generateDelayBody(action: Action, port: VarRef): String {
        return "actions.${action.name}.schedule(0, ${ASTUtils.generateVarRef(port)} as ${getActionType(action)});"
    }

    override fun generateForwardBody(action: Action, port: VarRef): String {
        return "${ASTUtils.generateVarRef(port)} = ${action.name} as ${getActionType(action)};"
    }

    override fun generateDelayGeneric(): String {
        return "T extends Present"
    }

    override fun generateAfterDelaysWithVariableWidth() = false
}
