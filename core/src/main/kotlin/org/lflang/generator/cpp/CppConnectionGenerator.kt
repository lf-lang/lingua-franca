package org.lflang.generator.cpp

import org.lflang.*
import org.lflang.generator.cpp.CppConnectionGenerator.Companion.cppType
import org.lflang.generator.cpp.CppConnectionGenerator.Companion.isEnclaveConnection
import org.lflang.generator.cpp.CppInstanceGenerator.Companion.isEnclave
import org.lflang.generator.cpp.CppPortGenerator.Companion.dataType
import org.lflang.lf.Connection
import org.lflang.lf.Port
import org.lflang.lf.Reactor
import org.lflang.lf.VarRef

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
                val leftPort = leftPorts.first()
                return when {
                    isEnclaveConnection -> when {
                        isPhysical    -> "reactor::PhysicalEnclaveConnection<${leftPort.dataType}>"
                        delay != null -> "reactor::DelayedEnclaveConnection<${leftPort.dataType}>"
                        else          -> "reactor::EnclaveConnection<${leftPort.dataType}>"
                    }

                    isPhysical          -> "reactor::PhysicalConnection<${leftPort.dataType}>"
                    delay != null       -> "reactor::DelayedConnection<${leftPort.dataType}>"
                    else                -> throw IllegalArgumentException("Unsupported connection type")
                }
            }

        val Connection.isEnclaveConnection: Boolean
            get() {
                for (port in leftPorts + rightPorts) {
                    if (port.container?.isEnclave == true) {
                        return true
                    }
                }
                return false
            }

        val Connection.requiresConnectionClass: Boolean get() = isPhysical || delay != null || isEnclaveConnection;
    }

    fun generateDeclarations() =
        reactor.connections.mapNotNull { generateDecleration(it) }
            .joinToString("\n", "// connections \n", postfix = "\n")

    fun generateInitializers() =
        reactor.connections.mapNotNull { generateConstructorInitializer(it) }.joinLn()

    private fun generateDecleration(connection: Connection): String? =
        with(connection) {
            if (requiresConnectionClass) {
                if (hasMultipleConnections) {
                    "std::vector<std::unique_ptr<${connection.cppType}>> ${connection.name};"
                } else {
                    "${connection.cppType} ${connection.name};"
                }
            } else null
        }

    private fun generateConstructorInitializer(connection: Connection): String? = with(connection) {
        if (requiresConnectionClass && !hasMultipleConnections) {
            when {
                isEnclaveConnection && delay == null -> """, $name{"$name", ${rightPorts[0].container.name}->__lf_env.get()}"""
                isEnclaveConnection && delay != null -> """, $name{"$name", ${rightPorts[0].container.name}->__lf_env.get(), ${delay.toCppTime()}}"""
                else                                 -> """, $name{"$name", this, ${delay.toCppTime()}}"""
            }
        } else null
    }

}
