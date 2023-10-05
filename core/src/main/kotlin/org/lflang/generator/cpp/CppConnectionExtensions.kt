package org.lflang.generator.cpp

import org.lflang.generator.cpp.CppInstanceGenerator.Companion.isEnclave
import org.lflang.lf.Connection
import org.lflang.lf.Reactor


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
        return when {
            isEnclaveConnection -> when {
                isPhysical -> "reactor::ConnectionType::PhysicalEnclaved"
                delay != null -> "reactor::ConnectionType::DelayedEnclaved"
                else -> "reactor::ConnectionType::Enclaved"
            }

            isPhysical -> "reactor::ConnectionType::Physical"
            delay != null -> "reactor::ConnectionType::Delayed"
            else -> "reactor::ConnectionType::Normal"
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
