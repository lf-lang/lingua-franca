package org.lflang.generator.ts

import org.lflang.isMultiport
import org.lflang.lf.Input
import org.lflang.lf.Output
import java.util.*

/**
 * Generate input and output ports for TypeScript target.
 */
class TSPortGenerator (
    private val inputs: List<Input>,
    private val outputs: List<Output>
) {

    fun generateClassProperties(): String {
        val portClassProperties = LinkedList<String>()
        for (input in inputs) {
            if (input.isMultiport) {
                portClassProperties.add("${input.name}: __InMultiPort<${input.tsPortType}>;")
            } else {
                portClassProperties.add("${input.name}: __InPort<${input.tsPortType}>;")
            }
        }
        for (output in outputs) {
            if (output.isMultiport) {
                portClassProperties.add("${output.name}: __OutMultiPort<${output.tsPortType}>;")
            } else {
                portClassProperties.add("${output.name}: __OutPort<${output.tsPortType}>;")
            }
        }
        return portClassProperties.joinToString("\n")
    }

    fun generateInstantiations(): String {
        val porInstantiations = LinkedList<String>()
        for (input in inputs) {
            if (input.isMultiport) {
                porInstantiations.add(
                    "this.${input.name} = new __InMultiPort<${input.tsPortType}>(this, ${input.widthSpec.toTSCode()});")
            } else {
                porInstantiations.add("this.${input.name} = new __InPort<${input.tsPortType}>(this);")
            }
        }
        for (output in outputs) {
            if (output.isMultiport) {
                porInstantiations.add(
                    "this.${output.name} = new __OutMultiPort<${output.tsPortType}>(this, ${output.widthSpec.toTSCode()});")
            } else {
                porInstantiations.add("this.${output.name} = new __OutPort<${output.tsPortType}>(this);")
            }
        }
        return porInstantiations.joinToString("\n")
    }
}
