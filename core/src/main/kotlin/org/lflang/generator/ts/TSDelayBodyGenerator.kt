package org.lflang.generator.ts

import org.lflang.ast.ASTUtils
import org.lflang.generator.DelayBodyGenerator
import org.lflang.lf.Action
import org.lflang.lf.VarRef

object TSDelayBodyGenerator : DelayBodyGenerator {
    /**
     * Return a TS type for the specified action.
     * If the type has not been specified, return
     * `unknown`.
     * @param action The action
     * @return The TS type.
     */
    private fun getActionType(action: Action): String {
        return if (action.type != null) {
            TSTypes.getInstance().getTargetType(action.type)
        } else {
            "unknown"
        }
    }

    override fun generateDelayBody(action: Action, port: VarRef): String {
        return "actions.${action.name}.schedule(0, ${ASTUtils.generateVarRef(port)} as ${getActionType(action)});"
    }

    override fun generateForwardBody(action: Action, port: VarRef): String {
        return "${ASTUtils.generateVarRef(port)} = ${action.name} as ${getActionType(action)};"
    }

    override fun generateDelayGeneric(): String {
        return "T"
    }

    override fun generateAfterDelaysWithVariableWidth() = false
}
