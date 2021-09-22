package org.lflang.generator.ts

import org.lflang.lf.Input
import org.lflang.lf.Output
import org.lflang.lf.Port
import org.lflang.lf.Type
import java.util.*

/**
 * Generate input and output ports for TypeScript target.
 */
class TSPortGenerator (
    // TODO(hokeun): Remove dependency on TSGenerator.
    private val tsGenerator: TSGenerator,
    private val inputs: List<Input>,
    private val outputs: List<Output>
) {
    private fun Type.getTargetType(): String = tsGenerator.getTargetTypeW(this)

    /**
     * Return a TS type for the specified port.
     * If the type has not been specified, return
     * "Present" which is the base type for ports.
     * @param port The port
     * @return The TS type.
     */
    private fun getPortType(port: Port): String {
        if (port.type != null) {
            return port.type.getTargetType()
        } else {
            return "Present"
        }
    }

    fun generateClassProperties(): String {
        val portClassProperties = LinkedList<String>()
        for (input in inputs) {
            portClassProperties.add("${input.name}: __InPort<${getPortType(input)}>;")
        }
        for (output in outputs) {
            portClassProperties.add("${output.name}: __OutPort<${getPortType(output)}>;")
        }
        return portClassProperties.joinToString("\n")
    }

    fun generateInstantiations(): String {
        val porInstantiations = LinkedList<String>()
        for (input in inputs) {
            porInstantiations.add("this.${input.name} = new __InPort<${getPortType(input)}>(this);")
        }
        for (output in outputs) {
            porInstantiations.add("this.${output.name} = new __OutPort<${getPortType(output)}>(this);")
        }
        return porInstantiations.joinToString("\n")
    }
}