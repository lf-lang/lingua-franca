package org.lflang.generator.ts

import org.lflang.ASTUtils
import org.lflang.lf.StateVar
import java.util.*

/**
 * Generator for state variables in TypeScript target.
 */
class TSStateGenerator (
    private val tsGenerator: TSGenerator,
    private val stateVars: List<StateVar>
) {
    private fun StateVar.getTargetType(): String = tsGenerator.getTargetTypeW(this)

    fun generateClassProperties(): String {
        val stateClassProperties = LinkedList<String>()
        for (stateVar in stateVars) {
            stateClassProperties.add("${stateVar.name}: __State<${stateVar.getTargetType()}>;");
        }
        return stateClassProperties.joinToString("\n")
    }

    private fun getInitializerList(state: StateVar): List<String> =
        tsGenerator.getInitializerListW(state)

    private fun getTargetInitializer(state: StateVar): String {
        return getInitializerList(state).joinToString(",")
    }
    fun generateInstantiations(): String {
        val stateInstantiations = LinkedList<String>()
        // Next handle states.
        for (stateVar in stateVars) {
            if (ASTUtils.isInitialized(stateVar)) {
                stateInstantiations.add("this.${stateVar.name} = new __State(${getTargetInitializer(stateVar)});");
            } else {
                stateInstantiations.add("this.${stateVar.name} = new __State(undefined);");
            }
        }
        return stateInstantiations.joinToString("\n")
    }
}