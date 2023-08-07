package org.lflang.generator.cpp

import org.lflang.capitalize
import org.lflang.lf.Reactor
import org.lflang.inferredType


val Reactor.allCppMessageTypes: Set<ROSMsgType>
    get() = with (this ){
        val s = inputs.map{ROSMsgType(it.inferredType.cppType)}.toMutableSet()
        s.addAll(outputs.map{ROSMsgType(it.inferredType.cppType)})
        return s
    }

data class ROSMsgType (private val _cppUserType : String){

    val cppUserType : String
        get() = _cppUserType

    val wrappedMsgCppInclude : String
        get() {
            // std_msgs have an extra "_" which needs to be removed, ROS message names must follow this regex: '^[A-Z][A-Za-z0-9]*$'
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
            return cppUserType.replace("::", "").replace("_", "").capitalize() + "Wrapped.msg"
        }
}

