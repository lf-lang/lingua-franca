package org.lflang.generator.ts

import org.lflang.ErrorReporter
import org.lflang.lf.Connection
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

    fun generateInstantiations(): String {
        val connectionInstantiations = LinkedList<String>()
        for (connection in connections) {
            var leftPortName = ""
            // FIXME: Add support for multiports.
            if (connection.leftPorts.size > 1) {
                errorReporter.reportError(connection, "Multiports are not yet supported in the TypeScript target.")
            } else {
                if (connection.leftPorts[0].container != null) {
                    leftPortName += connection.leftPorts[0].container.name + "."
                }
                leftPortName += connection.leftPorts[0].variable.name
            }
            var rightPortName = ""
            if (connection.rightPorts.size > 1) {
                errorReporter.reportError(connection, "Multiports are not yet supported in the TypeScript target.")
            } else {
                if (connection.rightPorts[0].container != null) {
                    rightPortName += connection.rightPorts[0].container.name + "."
                }
                rightPortName += connection.rightPorts[0].variable.name
            }
            if (leftPortName != "" && rightPortName != "") {
                connectionInstantiations.add("this._connect(this.$leftPortName, this.$rightPortName);")
            }
        }
        return connectionInstantiations.joinToString("\n")
    }
}