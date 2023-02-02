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
    }

    fun generateDeclarations() =
        reactor.connections.mapNotNull { generateDecleration(it) }
            .joinToString("\n", "// connections \n", postfix = "\n")

    fun generateInitializers() =
        reactor.connections.mapNotNull { generateConstructorInitializer(it) }.joinLn()

    private fun generateDecleration(connection: Connection): String? =
        when {
            connection.delay != null -> generateDelayedConnectionDeclaration(connection)
            else                     -> null
        }

    private fun generateConstructorInitializer(connection: Connection): String? =
        if (connection.delay != null && !connection.hasMultipleConnections)
            generateDelayedConnectionInitilizer(connection)
        else null

    private fun generateDelayedConnectionDeclaration(connection: Connection): String {
        val leftRef = connection.leftPorts.first()
        val leftPort = leftRef.variable as Port
        val dataType = leftPort.inferredType.cppType

        return if (connection.hasMultipleConnections) {
            "std::vector<reactor::DelayedConnection<$dataType>> ${connection.name};"
        } else {
            "reactor::DelayedConnection<$dataType> ${connection.name};"
        }
    }

    private fun generateDelayedConnectionInitilizer(connection: Connection): String {
        assert(!connection.hasMultipleConnections)
        val delay = connection.delay.toCppTime()
        val name = connection.name

        return """, $name{"$name", this, $delay}"""
    }
}