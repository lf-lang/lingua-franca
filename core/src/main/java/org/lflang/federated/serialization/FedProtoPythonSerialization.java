package org.lflang.federated.serialization;

import org.lflang.generator.GeneratorBase;
import org.lflang.target.Target;

/**
 * Enables support for Protocol Buffer serialization in Python federated programs.
 *
 * <p>In the Python target, protobuf serialization is performed by the user in Python code (e.g.,
 * using {@code SerializeToString()} and {@code ParseFromString()}). This class bridges the C level:
 * it extracts raw bytes from a Python {@code bytes} object on the sender side and reconstructs a
 * Python {@code bytes} object from raw network bytes on the receiver side.
 *
 * @author Hokeun Kim
 * @ingroup Federated
 */
public class FedProtoPythonSerialization implements FedSerialization {

  private static final String BUF_VAR = serializedVarName + "_buf";
  private static final String LEN_VAR = serializedVarName + "_len";

  @Override
  public boolean isCompatible(GeneratorBase generator) {
    if (generator.getTarget() != Target.Python) {
      throw new UnsupportedOperationException(
          "FedProtoPythonSerialization only supports the Python target.");
    }
    return true;
  }

  @Override
  public String serializedBufferLength() {
    return "(size_t)" + LEN_VAR;
  }

  @Override
  public String serializedBufferVar() {
    return "(unsigned char*)" + BUF_VAR;
  }

  /**
   * Generate code that extracts raw bytes from a Python {@code bytes} object.
   *
   * @param varName The Python bytes object (e.g. {@code sendRef->value}).
   * @param originalType Unused.
   */
  @Override
  public StringBuilder generateNetworkSerializerCode(String varName, String originalType) {
    StringBuilder code = new StringBuilder();
    code.append("char* " + BUF_VAR + ";\n");
    code.append("Py_ssize_t " + LEN_VAR + ";\n");
    code.append(
        "if (PyBytes_AsStringAndSize("
            + varName
            + ", &"
            + BUF_VAR
            + ", &"
            + LEN_VAR
            + ") == -1) {\n");
    code.append("    if (PyErr_Occurred()) PyErr_Print();\n");
    code.append(
        "    lf_print_error_and_exit(\"Could not extract bytes from Python proto value for"
            + " serialization.\");\n");
    code.append("}\n");
    return code;
  }

  /**
   * Generate code that wraps raw network bytes in a Python {@code bytes} object.
   *
   * @param varName The action variable name (accessed as {@code varName->token->value}).
   * @param targetType Unused.
   */
  @Override
  public StringBuilder generateNetworkDeserializerCode(String varName, String targetType) {
    StringBuilder code = new StringBuilder();
    code.append(
        "PyObject* "
            + deserializedVarName
            + " = PyBytes_FromStringAndSize((char*)"
            + varName
            + "->token->value, "
            + varName
            + "->token->length);\n");
    code.append("if (" + deserializedVarName + " == NULL) {\n");
    code.append("    if (PyErr_Occurred()) PyErr_Print();\n");
    code.append(
        "    lf_print_error_and_exit(\"Could not create Python bytes object for proto"
            + " deserialization.\");\n");
    code.append("}\n");
    return code;
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
