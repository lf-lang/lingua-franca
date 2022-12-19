package org.lflang.generator.ts

import org.lflang.federated.FederateInstance
import org.lflang.generator.getTargetTimeExpr
import org.lflang.lf.Action
import org.lflang.lf.ParameterReference
import java.util.*

/**
 * Generator for actions in TypeScript target.
 */
class TSActionGenerator(
    private val actions: List<Action>,
    private val federate: FederateInstance
) {

    fun generateClassProperties(): String {
        val stateClassProperties = LinkedList<String>()
        for (action in actions) {
            // Shutdown actions are handled internally by the
            // TypeScript reactor framework. There would be a
            // duplicate action if we included the one generated
            // by LF.
            if (action.name != "shutdown") {
                stateClassProperties.add("${action.name}: __Action<${action.tsActionType}>;")
            }
        }
        return stateClassProperties.joinToString("\n")
    }

    fun generateInstantiations(networkMessageActions: List<Action>): String {
        val actionInstantiations = LinkedList<String>()
        for (action in actions) {
            // Shutdown actions are handled internally by the
            // TypeScript reactor framework. There would be a
            // duplicate action if we included the one generated
            // by LF.
            if (action.name != "shutdown") {
                var actionArgs = "this, __Origin." + action.origin
                if (action.minDelay != null) {
                    // Actions in the TypeScript target are constructed
                    // with an optional minDelay argument which defaults to 0.
                    actionArgs += if (action.minDelay is ParameterReference) {
                        ", " + (action.minDelay as ParameterReference).parameter.name
                    } else {
                        ", " + action.minDelay.toTsTime()
                    }
                }
                if (action in networkMessageActions){
                    actionInstantiations.add(
                        "this.${action.name} = new __FederatePortAction<${action.tsActionType}>($actionArgs);")
                } else {
                    actionInstantiations.add(
                        "this.${action.name} = new __Action<${action.tsActionType}>($actionArgs);")
                }
            }
        }
        return actionInstantiations.joinToString("\n")
    }
}
