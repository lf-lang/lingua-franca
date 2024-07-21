/*************
 * Copyright (c) 2024, The University of California at Berkeley.
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

import org.lflang.generator.GeneratorBase;
import org.lflang.target.Target;

/**
 * Enables support for custom serialization.
 *
 * @author Shulu Li
 */
public class FedCustomPythonSerialization implements FedSerialization {

  String customSerializerPackage;

  public FedCustomPythonSerialization(String customSerializerPackage) {
    this.customSerializerPackage = customSerializerPackage;
  }

  @Override
  public boolean isCompatible(GeneratorBase generator) {
    if (generator.getTarget() != Target.Python) {
      throw new UnsupportedOperationException("The FedCustomPythonSerialization class only supports the Python target.");
    }
    return true;
  }

  @Override
  public String serializedBufferLength() {
    return serializedVarName + ".len";
  }

  @Override
  public String serializedBufferVar() {
    return serializedVarName + ".buf";
  }

  private String initializeCustomSerializer() {
    return "if (self->custom_serializer == NULL) {\n"
        + "PyObject* pName = PyUnicode_DecodeFSDefault(\"%s\");\n"
            .formatted(customSerializerPackage)
        + "PyObject* pModule = PyImport_Import(pName);\n"
        + "Py_DECREF(pName);\n"
        + "if (PyErr_Occurred()) PyErr_Print();\n"
        + "if (pModule == NULL) lf_print_error_and_exit(\"Could not load the custom serializer module '%s'.\");\n"
            .formatted(customSerializerPackage)
        + "PyObject* SerializerClass = PyObject_GetAttrString(pModule, \"Serializer\");\n"
        + "if (SerializerClass == NULL) lf_print_error_and_exit(\"Could not find class 'Serializer' in module '%s'.\");\n"
            .formatted(customSerializerPackage)
        + "Py_DECREF(pModule);\n"
        + "self->custom_serializer = PyObject_CallObject(SerializerClass, NULL);\n"
        + "Py_DECREF(SerializerClass);\n"
        + "if (self->custom_serializer == NULL) lf_print_error_and_exit(\"Could not instantiate class 'Serializer' in module '%s'.\");\n"
            .formatted(customSerializerPackage)
        + "lf_print_log(\"Loaded custom serializer module %s.\\n\");\n"
            .formatted(customSerializerPackage)
        + "}\n";
  }

  @Override
  public StringBuilder generateNetworkSerializerCode(String varName, String originalType) {
    StringBuilder serializerCode = new StringBuilder();
    // Check that self->custom_serializer is not null
    serializerCode.append(this.initializeCustomSerializer());
    // Define the serialized PyObject
    serializerCode
        .append(
            "PyObject *serializer_serialize = PyObject_GetAttrString(self->custom_serializer,"
                + " \"serialize\");\n"
                + "PyObject *args = PyTuple_Pack(1, ")
        .append(varName)
        .append(");\n")
        .append(
            "PyObject *serialized_pyobject = PyObject_CallObject(serializer_serialize, args);\n")
        .append("if (serialized_pyobject == NULL) {\n")
        .append("    if (PyErr_Occurred()) PyErr_Print();\n")
        .append("    lf_print_error_and_exit(\"Could not serialize object.\");\n")
        .append("}\n")
        .append("Py_buffer ")
        .append(serializedVarName)
        .append(";\n")
        .append("int returnValue = PyBytes_AsStringAndSize(serialized_pyobject, (char**)&")
        .append(serializedVarName)
        .append(".buf, &")
        .append(serializedVarName)
        .append(".len);\n");
    serializerCode.append("Py_XDECREF(serializer_serialize);\n");
    serializerCode.append("Py_XDECREF(args);\n");
    // Error check
    serializerCode.append("if (returnValue == -1) {\n");
    serializerCode.append("    if (PyErr_Occurred()) PyErr_Print();\n");
    serializerCode.append(
        "    lf_print_error_and_exit(\"Could not serialize " + serializedVarName + ".\");\n");
    serializerCode.append("}\n");
    return serializerCode;
  }

  @Override
  public StringBuilder generateNetworkDeserializerCode(String varName, String targetType) {
    StringBuilder deserializerCode = new StringBuilder();
    deserializerCode.append(this.initializeCustomSerializer());
    // Convert the network message to a Python ByteArray
    deserializerCode
        .append("PyObject* message_byte_array = " + "PyBytes_FromStringAndSize((char*)")
        .append(varName)
        .append("->token->value, ")
        .append(varName)
        .append("->token->length);\n");
    // Deserialize using Custom Serializer
    deserializerCode.append(
        "PyObject *serializer_deserialize = PyObject_GetAttrString(self->custom_serializer,"
            + " \"deserialize\");\n"
            + "PyObject *args = PyTuple_Pack(1, message_byte_array);\n"
            + "PyObject *"
            + deserializedVarName
            + " = PyObject_CallObject(serializer_deserialize, args);\n");
    // Error check
    deserializerCode.append("if (" + deserializedVarName + " == NULL) {\n");
    deserializerCode.append("    if (PyErr_Occurred()) PyErr_Print();\n");
    deserializerCode.append(
        "    lf_print_error_and_exit(\"Could not deserialize " + deserializedVarName + ".\");\n");
    deserializerCode.append("}\n");

    // Decrement the reference count
    deserializerCode.append("Py_XDECREF(serializer_deserialize);\n");
    deserializerCode.append("Py_XDECREF(message_byte_array);\n");
    deserializerCode.append("Py_XDECREF(args);\n");
    return deserializerCode;
  }

  @Override
  public StringBuilder generatePreambleForSupport() {
    return new StringBuilder();
  }

  @Override
  public StringBuilder generateCompilerExtensionForSupport() {
    return new StringBuilder();
  }
}
