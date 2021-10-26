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
 * @author Soroush Bateni <soroush@utdallas.edu>
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
        return "PyBytes_Size("+serializedVarName+")";
    }

    @Override
    public String seializedBufferVar() {
        return "PyBytes_AsString("+serializedVarName+")";
    }

    @Override
    public StringBuilder generateNetworkSerializerCode(String varName,
            String originalType) {
        StringBuilder serializerCode = new StringBuilder();
        
        // Define the serialized PyObject
        serializerCode.append("PyObject* "+serializedVarName+";\n");
        // Pickle the original PyObject
        serializerCode.append(serializedVarName+" = PyObject_CallMethod(global_pickler, \"dumps\", \"O\", "+varName+");\n");
            
        return serializerCode;
    }

    @Override
    public StringBuilder generateNetworkDeserializerCode(String varName,
            String targetType) {
        StringBuilder deserializerCode = new StringBuilder();
        
        deserializerCode.append("PyObject* "+deserializedVarName+" = PyObject_CallMethod(global_pickler, \"loads\", \"O\", "+varName+");\n");
        
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
