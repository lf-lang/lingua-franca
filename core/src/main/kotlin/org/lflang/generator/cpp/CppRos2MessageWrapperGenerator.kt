package org.lflang.generator.cpp

import org.lflang.capitalize
import org.lflang.joinWithLn

class CppRos2MessageWrapperGenerator (private val messageTypesToWrap : Set<ROSMsgType>){
    val ROSMsgType.fileContent : String
            get() {
                return """
                |lf_msgs_ros/Tag tag
                |${"$userTypeMsgInclude message"}
                """.trimMargin()
            }

    val fileContents : List<String>
        get() {
            return messageTypesToWrap.map{it.fileContent}
        }

    fun generateMessageFiles() : List<Pair<String, String>> {
        return messageTypesToWrap.map{
            Pair(
                it.wrappedMsgFileName,
                it.fileContent
            )
        }

    }
    fun generatePackageCmake(): String {
        val S = '$'
        messageTypesToWrap.forEach{ println(it.cppUserType) }
        val rosidl_generate_interfaces = if (messageTypesToWrap.isEmpty()) "" else { """
            |rosidl_generate_interfaces($S{PROJECT_NAME}
            | ${messageTypesToWrap.joinWithLn{"\"msg/${it.wrappedMsgFileName}\""}}
            |DEPENDENCIES std_msgs lf_msgs_ros
            |)"""
        }
        return """
        |cmake_minimum_required(VERSION 3.5)
        |project(lf_wrapped_msgs)
        |
        |find_package(rosidl_default_generators REQUIRED)
        |find_package(lf_msgs_ros REQUIRED)
        |find_package(std_msgs REQUIRED)
        |
        |
        |$rosidl_generate_interfaces
        |
        |ament_package()
        """.trimMargin()

    }

    fun generatePackageXml(): String {
        return """
            |<?xml version="1.0"?>
            |<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
            |<package format="3">
            |  <name>lf_wrapped_msgs</name>
            |  <version>0.0.0</version>
            |  <description>Generated message wrappers including original message type and a LF-tag</description>
            |  <maintainer email="todo@todo.com">Todo</maintainer>
            |  <license>Todo</license>
            |  
            |  <build_depend>rosidl_default_generators</build_depend>
            |  <depend>std_msgs</depend>
            |  <depend>lf_msgs_ros</depend>
            |  <exec_depend>rosidl_default_runtime</exec_depend>
            |
            |  <member_of_group>rosidl_interface_packages</member_of_group>
            |  <export>
            |    <build_type>ament_cmake</build_type>
            |  </export>
            |   
            |</package>
        """.trimMargin()
    }
}