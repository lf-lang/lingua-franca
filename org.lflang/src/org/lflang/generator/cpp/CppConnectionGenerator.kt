package org.lflang.generator.cpp

import org.lflang.*
import org.lflang.lf.Connection
import org.lflang.lf.Port
import org.lflang.lf.Reactor

class CppConnectionGenerator(private val reactor: Reactor) {

    companion object {
        val Connection.name: String
            get() =
                "connection_" + leftPorts.joinToString("__") {
                    if (it.container == null) {
                        it.variable.name
                    } else {
                        it.container.name + "_" + it.variable.name
                    }
                }

        val Connection.cppType: String
            get() {
                val leftPort = leftPorts.first().variable as Port
                val dataType = leftPort.inferredType.cppType

                return when {
                    isPhysical    -> "reactor::PhysicalConnection<$dataType>"
                    delay != null -> "reactor::DelayedConnection<$dataType>"
                    else          -> throw IllegalArgumentException("Connection is neither physical nor delayed")
                }
            }

        val Connection.requiresConnectionClass: Boolean get() = isPhysical || delay != null
    }

    fun generateDeclarations() =
        reactor.connections.mapNotNull { generateDecleration(it) }
            .joinToString("\n", "// connections \n", postfix = "\n")

    fun generateInitializers() =
        reactor.connections.mapNotNull { generateConstructorInitializer(it) }.joinLn()

    private fun generateDecleration(connection: Connection): String? =
        if (connection.requiresConnectionClass) {
            if (connection.hasMultipleConnections) {
                "std::vector<${connection.cppType}> ${connection.name};"
            } else {
                "${connection.cppType} ${connection.name};"
            }
        } else null

    private fun generateConstructorInitializer(connection: Connection): String? =
        if (connection.requiresConnectionClass && !connection.hasMultipleConnections) {
            val delay = connection.delay.toCppTime()
            val name = connection.name
            """, $name{"$name", this, $delay}"""
        } else null
}