package org.lflang.generator.ts

import org.lflang.ASTUtils
import org.lflang.generator.getTargetInitializer
import org.lflang.lf.StateVar
import java.util.*

/**
 * Generator for state variables in TypeScript target.
 */
class TSStateGenerator(
    private val stateVars: List<StateVar>
) {

    fun generateClassProperties(): String {
        val stateClassProperties = LinkedList<String>()
        for (stateVar in stateVars) {
            stateClassProperties.add("${stateVar.name}: __State<${TSTypes.getTargetType(stateVar)}>;");
        }
        return stateClassProperties.joinToString("\n")
    }

    fun generateInstantiations(): String {
        val stateInstantiations = LinkedList<String>()
        // Next handle states.
        for (stateVar in stateVars) {
            if (ASTUtils.isInitialized(stateVar)) {
                stateInstantiations.add("this.${stateVar.name} = new __State(${TSTypes.getTargetInitializer(stateVar)});");
            } else {
                stateInstantiations.add("this.${stateVar.name} = new __State(undefined);");
            }
        }
        return stateInstantiations.joinToString("\n")
    }
}
