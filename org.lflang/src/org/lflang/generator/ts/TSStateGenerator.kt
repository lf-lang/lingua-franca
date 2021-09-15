package org.lflang.generator.ts

import org.lflang.ASTUtils
import org.lflang.generator.PrependOperator
import org.lflang.lf.Reactor
import org.lflang.lf.StateVar
import java.util.*

class TSStateGenerator (
    private val tsGenerator: TSGenerator,
    private val reactor: Reactor
) {
    private fun StateVar.getTargetType(): String = tsGenerator.getTargetTypeW(this)

    fun generateClassProperties(): String {
        val stateClassProperties = LinkedList<String>()
        for (stateVar in reactor.stateVars) {
            stateClassProperties.add("${stateVar.name}: __State<${stateVar.getTargetType()}>;");
        }
        return with(PrependOperator) {
            """
            ${" |"..stateClassProperties.joinToString("\n")}
            """.trimMargin()
        }
    }

    private fun getInitializerList(state: StateVar): List<String> =
        tsGenerator.getInitializerListW(state)

    private fun getTargetInitializer(state: StateVar): String {
        return getInitializerList(state).joinToString(",")
    }
    fun generateInstantiations(): String {
        val stateInstantiations = LinkedList<String>()
        // Next handle states.
        for (stateVar in reactor.stateVars) {
            if (ASTUtils.isInitialized(stateVar)) {
                stateInstantiations.add("this.${stateVar.name} = new __State(${getTargetInitializer(stateVar)});");
            } else {
                stateInstantiations.add("this.${stateVar.name} = new __State(undefined);");
            }
        }
        return with(PrependOperator) {
            """
            ${" |"..stateInstantiations.joinToString("\n")}
            """.trimMargin()
        }
    }
}