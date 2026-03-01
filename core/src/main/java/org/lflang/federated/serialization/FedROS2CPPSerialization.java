package org.lflang.federated.serialization;

import org.lflang.generator.GeneratorBase;
import org.lflang.target.Target;
import org.lflang.target.property.CompilerProperty;

/**
 * Enables support for ROS 2 serialization in C/C++ code.
 *
 * @author Soroush Bateni
 * @ingroup Federated
 */
public class FedROS2CPPSerialization implements FedSerialization {

  /**
   * Check whether the current generator is compatible with the given serialization technique or
   * not.
   *
   * @param generator The current generator.
   * @return true if compatible, false if not.
   */
  @Override
  public boolean isCompatible(GeneratorBase generator) {
    if (generator.getTarget() != Target.C) {
      generator
          .messageReporter
          .nowhere()
          .error("ROS serialization is currently only supported for the C target.");
      return false;
    } else if (!generator
        .getTargetConfig()
        .get(CompilerProperty.INSTANCE)
        .equalsIgnoreCase("g++")) {
      generator
          .messageReporter
          .nowhere()
          .error("Please use the 'compiler: \"g++\"' target property \n" + "for ROS serialization");
      return false;
    }
    return true;
  }

  /**
   * @return Expression in target language that corresponds to the length of the serialized buffer.
   */
  @Override
  public String serializedBufferLength() {
    return serializedVarName + ".size()";
  }

  /**
   * @return Expression in target language that is the buffer variable itself.
   */
  @Override
  public String serializedBufferVar() {
    return serializedVarName + ".get_rcl_serialized_message().buffer";
  }

  /**
   * Generate code in C++ that serializes 'varName'. This code will convert the data in 'varName'
   * from its 'originalType' into an uint8_t. The serialized data will be put in a variable called
   * 'serialized_message', defined by @see serializedVarName.
   *
   * @param varName The variable to be serialized.
   * @param originalType The original type of the variable.
   * @return Target code that serializes the 'varName' from 'type' to an unsigned byte array.
   */
  @Override
  public StringBuilder generateNetworkSerializerCode(String varName, String originalType) {
    StringBuilder serializerCode = new StringBuilder();

    serializerCode.append("rclcpp::SerializedMessage " + serializedVarName + "(0u);\n");
    // Use the port type verbatim here, which can result
    // in compile error if it is not a valid ROS type
    serializerCode.append("using MessageT = ").append(originalType).append(";\n");
    serializerCode.append("static rclcpp::Serialization<MessageT> _lf_serializer;\n");
    serializerCode
        .append("_lf_serializer.serialize_message(&")
        .append(varName)
        .append("->value , &")
        .append(serializedVarName)
        .append(");\n");

    return serializerCode;
  }

  /**
   * Variant of @see generateNetworkSerializerCode(String varName, String originalType) that also
   * supports shared pointer (i.e., std::shared_ptr<>) definitions of ROS port types.
   *
   * @param varName The variable name.
   * @param originalType The original type name.
   * @param isSharedPtrType Indicates whether the port type is a shared pointer or not.
   */
  public StringBuilder generateNetworkSerializerCode(
      String varName, String originalType, boolean isSharedPtrType) {
    StringBuilder serializerCode = new StringBuilder();

    serializerCode.append("rclcpp::SerializedMessage " + serializedVarName + "(0u);\n");
    // Use the port type verbatim here, which can result
    // in compile error if it is not a valid ROS type
    serializerCode.append("using MessageT = ").append(originalType).append(";\n");
    serializerCode.append("static rclcpp::Serialization<MessageT> _lf_serializer;\n");
    if (isSharedPtrType) {
      serializerCode
          .append("_lf_serializer.serialize_message(")
          .append(varName)
          .append("->value.get() , &")
          .append(serializedVarName)
          .append(");\n");
    } else {
      serializerCode
          .append("_lf_serializer.serialize_message(&")
          .append(varName)
          .append("->value , &")
          .append(serializedVarName)
          .append(");\n");
    }

    return serializerCode;
  }

  /**
   * Generate code in C++ that deserializes 'varName'. This code will convert the data in 'varName'
   * from a uint8_t into the 'targetType'. The deserialized data will be put in a variable called
   * deserialized_message defined by @see deserializedVarName.
   *
   * @param varName The variable to deserialize.
   * @param targetType The type to deserialize into.
   * @return Target code that deserializes 'varName' from an unsigned byte array to 'type'.
   */
  @Override
  public StringBuilder generateNetworkDeserializerCode(String varName, String targetType) {
    StringBuilder deserializerCode = new StringBuilder();

    deserializerCode
        .append(
            "auto _lf_message = std::make_unique<rcl_serialized_message_t>("
                + " rcl_serialized_message_t{\n"
                + "    .buffer = (uint8_t*)")
        .append(varName)
        .append(".tmplt.token->value,\n")
        .append("    .buffer_length = ")
        .append(varName)
        .append(".tmplt.token->length,\n")
        .append("    .buffer_capacity = ")
        .append(varName)
        .append(".tmplt.token->length,\n")
        .append("    .allocator = rcl_get_default_allocator()\n")
        .append("});\n");
    deserializerCode.append(
        "auto _lf_msg ="
            + " std::make_unique<rclcpp::SerializedMessage>(std::move(*_lf_message.get()));\n");
    deserializerCode
        .append(varName)
        .append(".tmplt.token->value = NULL; // Manually move the data\n");
    // Use the port type verbatim here, which can result
    // in compile error if it is not a valid ROS type
    deserializerCode.append("using MessageT = ").append(targetType).append(";\n");
    deserializerCode.append(
        "MessageT "
            + deserializedVarName
            + " = MessageT();\n"
            + "auto _lf_serializer = rclcpp::Serialization<MessageT>();\n"
            + "_lf_serializer.deserialize_message(_lf_msg.get(), &"
            + deserializedVarName
            + ");\n");

    return deserializerCode;
  }

  /**
   * @return Code in C that includes all the necessary preamble to enable support for ROS 2
   *     serialization.
   */
  @Override
  public StringBuilder generatePreambleForSupport() {
    StringBuilder preamble = new StringBuilder();

    preamble.append(
        """
        #include "rcutils/allocator.h"
        #include "rclcpp/rclcpp.hpp"
        #include "rclcpp/serialization.hpp"
        #include "rclcpp/serialized_message.hpp"
        """);

    return preamble;
  }

  /**
   * @return Code that should be appended to the CMakeLists.txt to enable support for ROS 2
   *     serialization.
   */
  @Override
  public StringBuilder generateCompilerExtensionForSupport() {
    StringBuilder cMakeExtension = new StringBuilder();

    cMakeExtension.append(
        """
        enable_language(CXX)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-write-strings -O2")

        find_package(ament_cmake REQUIRED)
        find_package(rclcpp REQUIRED)
        find_package(rclcpp_components REQUIRED)
        find_package(rcutils)
        find_package(rmw REQUIRED)

        ament_target_dependencies(${LF_MAIN_TARGET} PUBLIC rclcpp rmw)""");

    return cMakeExtension;
  }
}
