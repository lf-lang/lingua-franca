package org.lflang.federated;

import org.lflang.Target;
import org.lflang.generator.GeneratorBase;

public class FedROSCPPSerialization implements FedSerialization {

    /**
     * Check whether the current generator is compatible with the given
     * serialization technique or not.
     * 
     * @param generator The current generator.
     * @return true if compatible, false if not.
     */
    @Override
    public boolean isCompatible(GeneratorBase generator) {
        if (generator.getTarget() != Target.C) {
            generator.errorReporter.reportError("ROS serialization is currently only supported for the C target.");
            return false;
        } else if (!generator.getTargetConfig().compiler.equalsIgnoreCase("g++")) {
            generator.errorReporter.reportError(
                    "Please use the 'compiler: \"g++\"' target property \n"+
                    "for ROS serialization"
            );
            return false;
        }
        return true;
    }
    
    @Override
    public String serializedVarLength() {
        return serializedVarName+".size()";
    }

    @Override
    public String seializedVarBuffer() {
        return serializedVarName+".get_rcl_serialized_message().buffer";
    }
    
    @Override
    public StringBuilder generateNetworkSerialzerCode(String portName, String portType) {
        StringBuilder serializerCode = new StringBuilder();
        
        serializerCode.append("rclcpp::SerializedMessage "+serializedVarName+"(0u);\n");
        // Use the port type verbatim here, which can result
        // in compile error if it is not a valid ROS type
        serializerCode.append("using MessageT = "+portType+";\n");
        serializerCode.append("static rclcpp::Serialization<MessageT> serializer;\n");
        serializerCode.append("serializer.serialize_message(&"+portName+"->value , &"+serializedVarName+");\n");
        
        return serializerCode;
    }

    @Override
    public StringBuilder generateNetworkDeserializerCode(String portName, String portType) {
        StringBuilder deserializerCode = new StringBuilder();
        
        deserializerCode.append(
                "auto message = std::make_unique<rcl_serialized_message_t>( rcl_serialized_message_t{\n"
                + "    .buffer = (uint8_t*)"+portName+"->token->value,\n"
                + "    .buffer_length = "+portName+"->token->length,\n"
                + "    .buffer_capacity = "+portName+"->token->length,\n"
                + "    .allocator = rcl_get_default_allocator()\n"
                + "});\n"
        );
        deserializerCode.append("auto msg = std::make_unique<rclcpp::SerializedMessage>(std::move(*message.get()));\n");
        deserializerCode.append(portName+"->token->value = NULL; // Manually move the data\n");
        // Use the port type verbatim here, which can result
        // in compile error if it is not a valid ROS type
        deserializerCode.append("using MessageT = "+portType+";\n");
        deserializerCode.append(
                "MessageT "+deserializedVarName+";\n"
              + "auto serializer = rclcpp::Serialization<MessageT>();\n"
              + "serializer.deserialize_message(msg.get(), &"+deserializedVarName+");\n"
        );
        
        return deserializerCode;
    }

    @Override
    public StringBuilder generatePreambleForSupport() {
        StringBuilder preamble = new StringBuilder();
        
        preamble.append(
            "#include \"rcutils/allocator.h\"\n"
          + "#include \"rclcpp/rclcpp.hpp\"\n"
          + "#include \"rclcpp/serialization.hpp\"\n"
          + "#include \"rclcpp/serialized_message.hpp\"\n"
        );
        
        return preamble;        
    }

    @Override
    public StringBuilder generateCompilerExtensionForSupport() {
        StringBuilder cMakeExtension = new StringBuilder();
        
        cMakeExtension.append(
            "enable_language(CXX)\n"
          + "set(CMAKE_CXX_FLAGS \"${CMAKE_CXX_FLAGS} -Wno-write-strings -O2\")\n"
          + "\n"
          + "find_package(ament_cmake REQUIRED)\n"
          + "find_package(rclcpp REQUIRED)\n"
          + "find_package(rclcpp_components REQUIRED)\n"
          + "find_package(rcutils)\n"
          + "find_package(rmw REQUIRED)\n"
          + "\n"
          + "ament_target_dependencies( ${LF_MAIN_TARGET} rclcpp rmw)"
        );
        
        return cMakeExtension;
    }

}
