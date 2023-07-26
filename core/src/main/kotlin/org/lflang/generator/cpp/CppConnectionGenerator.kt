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
                        isPhysical    -> "reactor::ConnectionType::PhysicalEnclaved"
                        delay != null -> "reactor::ConnectionType::DelayedEnclaved"
                        else          -> "reactor::ConnectionType::Enclaved"
                    }

                    isPhysical          -> "reactor::ConnectionType::Physical"
                    delay != null       -> "reactor::ConnectionType::Delayed"
                    else                -> "reactor::ConnectionType::Normal"
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
    }

}
