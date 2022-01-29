package org.lflang.generator.ts

import org.lflang.generator.ts.TSTypes.getTargetType
import org.lflang.lf.Input
import org.lflang.lf.Output
import java.util.*

/**
 * Generate input and output ports for TypeScript target.
 */
class TSPortGenerator(
    private val inputs: List<Input>,
    private val outputs: List<Output>
) {

    fun generateClassProperties(): String {
        val portClassProperties = mutableListOf<String>()
        for (input in inputs) {
            portClassProperties.add("${input.name}: __InPort<${getTargetType(input.type)}>;")
        }
        for (output in outputs) {
            portClassProperties.add("${output.name}: __OutPort<${getTargetType(output.type)}>;")
        }
        return portClassProperties.joinToString("\n")
    }

    fun generateInstantiations(): String {
        val porInstantiations = mutableListOf<String>()
        for (input in inputs) {
            porInstantiations.add("this.${input.name} = new __InPort<${getTargetType(input.type)}>(this);")
        }
        for (output in outputs) {
            porInstantiations.add("this.${output.name} = new __OutPort<${getTargetType(output.type)}>(this);")
        }
        return porInstantiations.joinToString("\n")
    }
}
