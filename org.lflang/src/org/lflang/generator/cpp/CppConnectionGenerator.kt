package org.lflang.generator.cpp

import org.lflang.generator.cpp.CppConnectionGenerator.Companion.name
import org.lflang.inferredType
import org.lflang.joinLn
import org.lflang.joinWithLn
import org.lflang.lf.Connection
import org.lflang.lf.Port
import org.lflang.lf.Reactor
import org.lflang.lf.VarRef

class CppConnectionGenerator(private val reactor: Reactor) {

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
        when {
            connection.delay != null -> generateDelayedConnectionInitilizer(connection)
            else                     -> null
        }

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


    private fun generateDelayedConnectionDeclaration(connection: Connection): String {
        if (connection.leftPorts.size != 1) {
            TODO("After delays on multiports are not yet supported")
        }
        val leftRef = connection.leftPorts.first()
        val leftPort = leftRef.variable as Port
        val dataType = leftPort.inferredType.cppType

        return "reactor::DelayedConnection<$dataType> ${connection.name};"
    }

    private fun generateDelayedConnectionInitilizer(connection: Connection): String {
        if (connection.leftPorts.size != 1) {
            TODO("After delays on multiports are not yet supported")
        }
        if (connection.rightPorts.size != 1) {
            TODO("After delays on multiports are not yet supported")
        }
        val delay = connection.delay.toCppTime()
        val name = connection.name

        return """, $name{"$name}", this, $delay}"""
    }
}