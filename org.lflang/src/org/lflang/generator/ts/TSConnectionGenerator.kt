package org.lflang.generator.ts

import org.lflang.ErrorReporter
import org.lflang.hasMultipleConnections
import org.lflang.isBank
import org.lflang.isInput
import org.lflang.isMultiport
import org.lflang.lf.Connection
import org.lflang.lf.Port
import org.lflang.lf.Reactor
import org.lflang.lf.VarRef
import org.lflang.toText
import java.util.*

/**
 * A code generator for connections using _connect() method of a TypeScript reactor class
 * This class corresponds to CppAssembleMethodGenerator of C++ code generator.
 */
class TSConnectionGenerator (
    private val connections: List<Connection>,
    private val errorReporter: ErrorReporter
) {
    // There is no generateClassProperties() for connections

    private fun getPortTypeName(port: Port, genericTypeMap: HashMap<String, String>): String {
        var portType = getPortType(port)
        portType = genericTypeMap[portType]?: portType
        return (if(port.isInput) "__In" else "__Out") +
                "${if(port.isMultiport) "Multi" else ""}Port<$portType>"
    }

    private fun getPortName(port: VarRef, genericTypeMap: HashMap<String, String>): String {
        var portName = ""
        if (port.container != null) {
            if (port.container.isBank) {
                portName += "...this.${port.container.name}.all().reduce(" +
                        "(__acc, __val) => __acc.concat(__val.${port.variable.name}), " +
                        "new Array<${getPortTypeName(port.variable as Port, genericTypeMap)}>(0))"
                return portName
            }
            portName += port.container.name + "."
        }
        portName += port.variable.name
        return "this.$portName"
    }

    private fun getPortNames(ports: List<VarRef>): List<String> {
        var portNames = LinkedList<String>()
        var genericTypeMap: HashMap<String, String>? = null
        for (varRef in ports) {
            if (genericTypeMap == null) {
                genericTypeMap = HashMap<String, String>()
            }
            if (varRef.container != null) {
                for ((index, typeParam) in (varRef.container.reactorClass as Reactor).typeParms.withIndex()) {
                    genericTypeMap[typeParam.toText().split(" ")[0]] = varRef.container.typeParms[index].toText()
                }
            }
            portNames.add(getPortName(varRef, genericTypeMap))
        }
        return portNames
    }

    fun generateInstantiations(): String {
        val connectionInstantiations = LinkedList<String>()
        for (connection in connections) {
            if (connection.hasMultipleConnections) {
                var leftPortNames = getPortNames(connection.leftPorts)
                var rightPortNames = getPortNames(connection.rightPorts)
                val leftPortName = leftPortNames.joinToString(prefix = "[", separator = ", ", postfix = "]")
                val rightPortName = rightPortNames.joinToString(prefix = "[", separator = ", ", postfix = "]")
                connectionInstantiations.add("this._connectMulti($leftPortName, $rightPortName, ${connection.isIterated});")
            } else {
                var leftPortName = ""

                if (connection.leftPorts[0].container != null) {
                    leftPortName += connection.leftPorts[0].container.name + "."
                }
                leftPortName += connection.leftPorts[0].variable.name
                var rightPortName = ""

                if (connection.rightPorts[0].container != null) {
                    rightPortName += connection.rightPorts[0].container.name + "."
                }
                rightPortName += connection.rightPorts[0].variable.name
                if (leftPortName != "" && rightPortName != "") {
                    connectionInstantiations.add("this._connect(this.$leftPortName, this.$rightPortName);")
                }
            }
        }
        return connectionInstantiations.joinToString("\n")
    }
}