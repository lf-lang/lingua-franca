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
 * Enables support for Python pickle serialization.
 * 
 * @author Soroush Bateni
 *
 */
public class FedNativePythonSerialization implements FedSerialization {

    @Override
    public boolean isCompatible(GeneratorBase generator) {
        if (generator.getTarget() != Target.Python ) {
            throw new UnsupportedOperationException("This class only support Python serialization.");
        }
        return true;
    }

    @Override
    public String serializedBufferLength() {
        return serializedVarName+".len";
    }

    @Override
    public String seializedBufferVar() {
        return serializedVarName+".buf";
    }

    @Override
    public StringBuilder generateNetworkSerializerCode(String varName,
            String originalType) {
        StringBuilder serializerCode = new StringBuilder();

        // Check that global_pickler is not null
        serializerCode.append("if (global_pickler == NULL) lf_print_error_and_exit(\"The pickle module is not loaded.\");\n");
        // Define the serialized PyObject
        serializerCode.append("PyObject* serialized_pyobject = PyObject_CallMethod(global_pickler, \"dumps\", \"O\", "+varName+");\n");

        // Error check
        serializerCode.append("if (serialized_pyobject == NULL) {\n");
        serializerCode.append("    if (PyErr_Occurred()) PyErr_Print();\n");
        serializerCode.append("    lf_print_error_and_exit(\"Could not serialize serialized_pyobject.\");\n");
        serializerCode.append("}\n");
        
        serializerCode.append("Py_buffer "+serializedVarName+";\n");
        serializerCode.append("int returnValue = PyBytes_AsStringAndSize(serialized_pyobject, &"+serializedVarName+".buf, &"+serializedVarName+".len);\n");
        // Error check
        serializerCode.append("if (returnValue == -1) {\n");
        serializerCode.append("    if (PyErr_Occurred()) PyErr_Print();\n");
        serializerCode.append("    lf_print_error_and_exit(\"Could not serialize "+serializedVarName+".\");\n");
        serializerCode.append("}\n");
        
        
            
        return serializerCode;
    }

    @Override
    public StringBuilder generateNetworkDeserializerCode(String varName,
            String targetType) {
        StringBuilder deserializerCode = new StringBuilder();
        
        // Convert the network message to a Python ByteArray
        deserializerCode.append("PyObject* message_byte_array = "+
                "PyBytes_FromStringAndSize((char*)"+varName+"->token->value, "+varName+"->token->length);\n");
        // Deserialize using Pickle
        deserializerCode.append("Py_XINCREF(message_byte_array);\n");
        deserializerCode.append("PyObject* "+deserializedVarName+
                " = PyObject_CallMethod(global_pickler, \"loads\", \"O\", message_byte_array);\n");
        // Error check
        deserializerCode.append("if ("+deserializedVarName+" == NULL) {\n");
        deserializerCode.append("    if (PyErr_Occurred()) PyErr_Print();\n");
        deserializerCode.append("    lf_print_error_and_exit(\"Could not deserialize "+deserializedVarName+".\");\n");
        deserializerCode.append("}\n");
        
        // Decrment the reference count
        deserializerCode.append("Py_XDECREF(message_byte_array);\n");
        
        return deserializerCode;
    }

    @Override
    public StringBuilder generatePreambleForSupport() {
        return new StringBuilder("");
    }

    @Override
    public StringBuilder generateCompilerExtensionForSupport() {
        return new StringBuilder("");
    }

}
