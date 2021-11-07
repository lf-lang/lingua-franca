package org.lflang.generator.ts

import org.lflang.inferredType
import org.lflang.lf.Parameter
import java.util.*

/**
 * Generate parameters for TypeScript target.
 */
class TSParameterGenerator(
    private val parameters: List<Parameter>
) {

    fun generateClassProperties(): String {
        val paramClassProperties = LinkedList<String>()
        for (param in parameters) {
            paramClassProperties.add("${param.name}: __Parameter<${TsTypes.getTargetType(param.inferredType)}>;")
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
