package org.lflang.generator.cpp

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

    private fun delayedConnectionName(ref: VarRef) =
        when (ref.container) {
            null -> "connection_" + ref.variable.name
            else -> "connection_" + ref.container.name + "_" + ref.variable.name
        }

    private fun generateDelayedConnectionDeclaration(connection: Connection): String  {
        if (connection.leftPorts.size != 1) {
            TODO("After delays on multiports are not yet supported")
        }
        val leftRef = connection.leftPorts.first()
        val leftPort = leftRef.variable as Port
        val dataType = leftPort.inferredType.cppType
        val name = delayedConnectionName(leftRef)

        return "reactor::Connection<$dataType> $name;"
    }

    private fun generateDelayedConnectionInitilizer(connection: Connection): String  {
        if (connection.leftPorts.size != 1) {
            TODO("After delays on multiports are not yet supported")
        }
        if (connection.rightPorts.size != 1) {
            TODO("After delays on multiports are not yet supported")
        }
        val leftRef = connection.leftPorts.first()
        val rightRef = connection.leftPorts.first()
        val delay = connection.delay.toCppTime()
        val name = delayedConnectionName(leftRef)

        return """, $name{"$name}", this, $delay, &(${leftRef.name}), &(${rightRef.name})}"""
    }
}