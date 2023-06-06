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

class CppROS2ConnectionGenerator(private val reactor: Reactor) : ConnectionGenerator {
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
                    isSubscriberConnection -> "reactor::ROS2SubscriberConnection" //<${leftPort.dataType}>"
                    else                -> throw IllegalArgumentException("Unsupported connection type")
                }
            }

        val Connection.isSubscriberConnection: Boolean
            get() {
                return true
                for (port in leftPorts + rightPorts) {
                    if (port.container?.isEnclave == true) {
                        return true
                    }
                }
                return false
            }

    }

    override fun generateDeclarations() =
        reactor.connections.mapNotNull { generateDeclaration(it) }
            .joinToString("\n", "// connections \n", postfix = "\n")

    override fun generateInitializers() =
        reactor.connections.mapNotNull { generateConstructorInitializer(it) }.joinLn()

    private fun generateDeclaration(connection: Connection): String? =
        with(connection) {
            return null
            if (hasMultipleConnections) {
                "std::vector<std::unique_ptr<${connection.cppType}>> ${connection.name};"
            } else {
                "${connection.cppType} ${connection.name};"
            }
        }

    private fun generateConstructorInitializer(connection: Connection): String? = with(connection) {
        return null
        if (!hasMultipleConnections) {
            when {
                //isSubscriberConnection && delay == null -> """, $name{"$name", ${leftPorts[0].container.name}->__lf_env.get()}"""
                //isSubscriberConnection && delay != null -> """, $name{"$name", ${leftPorts[0].container.name}->__lf_env.get(), ${delay.toCppTime()}}"""
                //!isSubscriberConnection && delay == null -> """, $name{"$name", ${rightPorts[0].container.name}->__lf_env.get()}"""
                else                                 -> """, $name{"$name", this, ${delay.toCppTime()}}"""
            }
        } else null
    }

}
