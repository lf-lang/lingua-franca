package org.lflang.generator.cpp

import org.lflang.*
import org.lflang.ast.ASTUtils
import org.lflang.generator.cpp.CppInstanceGenerator.Companion.isEnclave
import org.lflang.generator.cpp.CppTypes.getTargetTimeExpr
import org.lflang.lf.*


val Reactor.allCppMessageTypes: Set<ROSMsgType>
    get() = with (this ){
        val reactors : MutableList<Reactor> = mutableListOf(this)
        val types : MutableSet<ROSMsgType> = mutableSetOf()
        while (reactors.isNotEmpty()) {
            val r = reactors.removeFirst()
            types.addAll(r.inputs.map{ROSMsgType(it.inferredType.cppType)})
            types.addAll(r.outputs.map{ROSMsgType(it.inferredType.cppType)})
            for (inst in r.instantiations) reactors.add(inst.reactor)
        }

        return types
    }

data class ROSMsgType (private val _cppUserType : String){

    val cppUserType : String
        get() = _cppUserType

    val wrappedMsgCppInclude : String
        get() {
            // std_msgs have an extra "_" which needs to be removed
            var msgT = cppUserType.replace("::", "")
            msgT = msgT.replace("_", "")
            msgT = msgT.replaceFirstChar(Char::lowercase)+ "Wrapped.hpp\""
            msgT = msgT.map{ if (it.isUpperCase()) "_${it.lowercase()}" else it}.joinToString("")
            return "#include \"lf_wrapped_msgs/msg/$msgT"
        }

    val userTypeMsgInclude : String
        get() {
            return cppUserType.replace("::", "/").replace("msg/", "")
        }

    val wrappedCppType : String
        get() {
            return "lf_wrapped_msgs::msg::" + cppUserType.replace("::", "").replace("_", "").capitalize() + "Wrapped"
        }


    val wrappedMsgFileName : String
        get() {
            // ROS message file names must follow this regex: '^[A-Z][A-Za-z0-9]*$'
            return cppUserType.replace("::", "").replace("_", "").capitalize() + "Wrapped.msg"
        }
}

val Connection.isFederateConnection: Boolean
    get() {
        for (port in leftPorts + rightPorts) {
            if (port.container != null && AttributeUtils.isFederate(port.container)) {
                return true
            }
        }
        return false
    }



