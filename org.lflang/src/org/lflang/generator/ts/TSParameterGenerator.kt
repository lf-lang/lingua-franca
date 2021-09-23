package org.lflang.generator.ts

import org.lflang.generator.PrependOperator
import org.lflang.lf.Parameter
import java.util.*

/**
 * Generate parameters for TypeScript target.
 */
class TSParameterGenerator (
    // TODO(hokeun): Remove dependency on TSGenerator.
    private val tsGenerator: TSGenerator,
    private val parameters: List<Parameter>
 ) {
    private fun Parameter.getTargetType(): String = tsGenerator.getTargetTypeW(this)

    fun generateClassProperties(): String {
        val paramClassProperties = LinkedList<String>()
        for (param in parameters) {
            paramClassProperties.add("${param.name}: __Parameter<${param.getTargetType()}>;")
        }
        return paramClassProperties.joinToString("\n")
    }

    fun generateInstantiations(): String {
        val paramInstantiations = LinkedList<String>()
        for (param in parameters) {
            paramInstantiations.add("this.${param.name} = new __Parameter(${param.name});")
        }
        return paramInstantiations.joinToString("\n")
    }
}