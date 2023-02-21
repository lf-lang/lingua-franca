/*************
 * Copyright (c) 2021, The University of California at Berkeley.
 * Copyright (c) 2021, The University of Texas at Dallas.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.federated.serialization;

import org.lflang.Target;
import org.lflang.generator.GeneratorBase;

/**
 * Enables support for ROS 2 serialization in C/C++ code.
 * 
 * @author Soroush Bateni
 *
 */
public class FedROS2CPPSerialization implements FedSerialization {

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
    
    /** 
     * @return Expression in target language that corresponds to the length
     *  of the serialized buffer.
     */
    @Override
    public String serializedBufferLength() {
        return serializedVarName+".size()";
    }

    /**
     * @return Expression in target language that is the buffer variable 
     *  itself.
     */
    @Override
    public String seializedBufferVar() {
        return serializedVarName+".get_rcl_serialized_message().buffer";
    }
    
    /**
     * Generate code in C++ that serializes 'varName'. This code
     * will convert the data in 'varName' from its 'originalType' into an 
     * uint8_t. The serialized data will be put in a variable called 
     * 'serialized_message', defined by @see serializedVarName.
     * 
     * @param varName The variable to be serialized.
     * @param originalType The original type of the variable.
     * @return Target code that serializes the 'varName' from 'type'
     *  to an unsigned byte array.
     */
    @Override
    public StringBuilder generateNetworkSerializerCode(String varName, String originalType) {
        StringBuilder serializerCode = new StringBuilder();
        
        serializerCode.append("rclcpp::SerializedMessage "+serializedVarName+"(0u);\n");
        // Use the port type verbatim here, which can result
        // in compile error if it is not a valid ROS type
        serializerCode.append("using MessageT = "+originalType+";\n");
        serializerCode.append("static rclcpp::Serialization<MessageT> _lf_serializer;\n");
        serializerCode.append("_lf_serializer.serialize_message(&"+varName+"->value , &"+serializedVarName+");\n");
        
        return serializerCode;
    }
    
    /**
     * Variant of @see generateNetworkSerializerCode(String varName, String originalType)
     * that also supports shared pointer (i.e., std::shared_ptr<>) definitions of ROS port
     * types.
     * @param isSharedPtrType Indicates whether the port type is a shared pointer or not.
     */
    public StringBuilder generateNetworkSerializerCode(String varName, String originalType, boolean isSharedPtrType) {
        StringBuilder serializerCode = new StringBuilder();
        
        serializerCode.append("rclcpp::SerializedMessage "+serializedVarName+"(0u);\n");
        // Use the port type verbatim here, which can result
        // in compile error if it is not a valid ROS type
        serializerCode.append("using MessageT = "+originalType+";\n");
        serializerCode.append("static rclcpp::Serialization<MessageT> _lf_serializer;\n");
        if (isSharedPtrType) {
            serializerCode.append("_lf_serializer.serialize_message("+varName+"->value.get() , &"+serializedVarName+");\n");
        } else {
            serializerCode.append("_lf_serializer.serialize_message(&"+varName+"->value , &"+serializedVarName+");\n");
        }
            
        return serializerCode;
    }


    
    /**
     * Generate code in C++ that deserializes 'varName'. This code will
     * convert the data in 'varName' from a uint8_t into the 'targetType'.
     * The deserialized data will be put in a variable called deserialized_message
     * defined by @see deserializedVarName.
     *  
     * @param varName The variable to deserialize.
     * @param targetType The type to deserialize into.
     * @return Target code that deserializes 'varName' from an unsigned byte array
     *  to 'type'.
     */
    @Override
    public StringBuilder generateNetworkDeserializerCode(String varName, String targetType) {
        StringBuilder deserializerCode = new StringBuilder();
        
        deserializerCode.append(
                "auto message = std::make_unique<rcl_serialized_message_t>( rcl_serialized_message_t{\n"
                + "    .buffer = (uint8_t*)"+varName+".tmplt.token->value,\n"
                + "    .buffer_length = "+varName+".tmplt.token->length,\n"
                + "    .buffer_capacity = "+varName+".tmplt.token->length,\n"
                + "    .allocator = rcl_get_default_allocator()\n"
                + "});\n"
        );
        deserializerCode.append("auto msg = std::make_unique<rclcpp::SerializedMessage>(std::move(*message.get()));\n");
        deserializerCode.append(varName+".tmplt.token->value = NULL; // Manually move the data\n");
        // Use the port type verbatim here, which can result
        // in compile error if it is not a valid ROS type
        deserializerCode.append("using MessageT = "+targetType+";\n");
        deserializerCode.append(
                "MessageT "+deserializedVarName+" = MessageT();\n"
              + "auto _lf_serializer = rclcpp::Serialization<MessageT>();\n"
              + "_lf_serializer.deserialize_message(msg.get(), &"+deserializedVarName+");\n"
        );
        
        return deserializerCode;
    }

    /**
     * @return Code in C that includes all the necessary preamble to enable
     *  support for ROS 2 serialization.
     */
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
    
    /** 
     * @return Code that should be appended to the CMakeLists.txt to enable 
     *  support for ROS 2 serialization.
     */
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
