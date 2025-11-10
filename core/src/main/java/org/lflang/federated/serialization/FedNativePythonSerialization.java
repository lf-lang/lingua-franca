package org.lflang.federated.serialization;

import org.lflang.generator.GeneratorBase;
import org.lflang.target.Target;

/**
 * Enables support for Python pickle serialization.
 *
 * @author Soroush Bateni
 * @ingroup Federated
 */
public class FedNativePythonSerialization implements FedSerialization {

  @Override
  public boolean isCompatible(GeneratorBase generator) {
    if (generator.getTarget() != Target.Python) {
      throw new UnsupportedOperationException("This class only support Python serialization.");
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

  @Override
  public StringBuilder generateNetworkSerializerCode(String varName, String originalType) {
    StringBuilder serializerCode = new StringBuilder();

    // Check that global_pickler is not null
    serializerCode.append(
        "if (global_pickler == NULL) lf_print_error_and_exit(\"The pickle module is not"
            + " loaded.\");\n");
    // Define the serialized PyObject
    serializerCode
        .append(
            "PyObject* serialized_pyobject = PyObject_CallMethod(global_pickler, \"dumps\", \"O\","
                + " ")
        .append(varName)
        .append(");\n");

    // Error check
    serializerCode.append("if (serialized_pyobject == NULL) {\n");
    serializerCode.append("    if (PyErr_Occurred()) PyErr_Print();\n");
    serializerCode.append(
        "    lf_print_error_and_exit(\"Could not serialize serialized_pyobject.\");\n");
    serializerCode.append("}\n");

    serializerCode.append("Py_buffer " + serializedVarName + ";\n");
    serializerCode.append(
        "int returnValue = PyBytes_AsStringAndSize(serialized_pyobject, (char**)&"
            + serializedVarName
            + ".buf, &"
            + serializedVarName
            + ".len);\n");
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

    // Convert the network message to a Python ByteArray
    deserializerCode
        .append("PyObject* message_byte_array = " + "PyBytes_FromStringAndSize((char*)")
        .append(varName)
        .append("->token->value, ")
        .append(varName)
        .append("->token->length);\n");
    // Deserialize using Pickle
    deserializerCode.append(
        "PyObject* "
            + deserializedVarName
            + " = PyObject_CallMethod(global_pickler, \"loads\", \"O\", message_byte_array);\n");
    // Error check
    deserializerCode.append("if (" + deserializedVarName + " == NULL) {\n");
    deserializerCode.append("    if (PyErr_Occurred()) PyErr_Print();\n");
    deserializerCode.append(
        "    lf_print_error_and_exit(\"Could not deserialize " + deserializedVarName + ".\");\n");
    deserializerCode.append("}\n");

    // Decrement the reference count
    deserializerCode.append("Py_XDECREF(message_byte_array);\n");

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
