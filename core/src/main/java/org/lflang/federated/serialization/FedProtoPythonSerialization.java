package org.lflang.federated.serialization;

import org.lflang.generator.GeneratorBase;
import org.lflang.target.Target;

/**
 * Enables support for Protocol Buffer serialization in Python federated programs.
 *
 * <p>User reactions work with proto objects directly (no manual SerializeToString /
 * ParseFromString calls). The generated C code:
 *
 * <ul>
 *   <li>Sender: calls {@code SerializeToString()} on the Python proto object and prepends the
 *       message's fully-qualified type name so the receiver can reconstruct the right class.
 *   <li>Receiver: reads the type name from the wire format, uses Python's {@code
 *       google.protobuf.symbol_database} to obtain the class, creates an instance, and calls
 *       {@code ParseFromString()}.
 * </ul>
 *
 * <p>Wire format: {@code [4-byte big-endian name length][UTF-8 full_name][proto bytes]}
 *
 * @author Hokeun Kim
 * @ingroup Federated
 */
public class FedProtoPythonSerialization implements FedSerialization {

  /** Name of the malloc'd wire-format buffer variable in generated C code. */
  public static final String BUF_VAR = "_lf_proto_wire_buf";

  /** Name of the wire-format buffer length variable in generated C code. */
  public static final String LEN_VAR = "_lf_proto_wire_len";

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
    return LEN_VAR;
  }

  @Override
  public String serializedBufferVar() {
    return BUF_VAR;
  }

  /**
   * Generate C code that serializes a Python proto object into a self-describing wire buffer.
   *
   * <p>Wire format: {@code [4-byte big-endian name_len][full_name bytes][proto bytes]}
   *
   * <p>The caller is responsible for calling {@code free(_lf_proto_wire_buf)} after the buffer has
   * been sent.
   *
   * @param varName C expression for the Python proto object (e.g. {@code sendRef->value}).
   * @param originalType Unused.
   */
  @Override
  public StringBuilder generateNetworkSerializerCode(String varName, String originalType) {
    StringBuilder code = new StringBuilder();
    // Serialize the proto object to bytes.
    code.append(
        "PyObject* _lf_proto_bytes = PyObject_CallMethod("
            + varName
            + ", \"SerializeToString\", NULL);\n");
    code.append("if (_lf_proto_bytes == NULL) {\n");
    code.append("    if (PyErr_Occurred()) PyErr_Print();\n");
    code.append(
        "    lf_print_warning(\"Failed to call SerializeToString on proto object."
            + " Dropping outgoing message.\");\n");
    code.append("    return;\n");
    code.append("}\n");
    code.append("char* _lf_proto_raw;\n");
    code.append("Py_ssize_t _lf_proto_raw_len;\n");
    code.append(
        "if (PyBytes_AsStringAndSize(_lf_proto_bytes, &_lf_proto_raw, &_lf_proto_raw_len)"
            + " == -1) {\n");
    code.append("    if (PyErr_Occurred()) PyErr_Print();\n");
    code.append("    Py_XDECREF(_lf_proto_bytes);\n");
    code.append(
        "    lf_print_warning(\"Failed to extract bytes from serialized proto."
            + " Dropping outgoing message.\");\n");
    code.append("    return;\n");
    code.append("}\n");
    // Get the fully-qualified type name from DESCRIPTOR.full_name.
    code.append(
        "PyObject* _lf_descriptor = PyObject_GetAttrString(" + varName + ", \"DESCRIPTOR\");\n");
    code.append("if (_lf_descriptor == NULL) {\n");
    code.append("    if (PyErr_Occurred()) PyErr_Print();\n");
    code.append("    lf_print_error_and_exit(\"Failed to get DESCRIPTOR from proto object.\");\n");
    code.append("}\n");
    code.append(
        "PyObject* _lf_full_name_obj = PyObject_GetAttrString(_lf_descriptor, \"full_name\");\n");
    code.append("Py_XDECREF(_lf_descriptor);\n");
    code.append("if (_lf_full_name_obj == NULL) {\n");
    code.append("    if (PyErr_Occurred()) PyErr_Print();\n");
    code.append("    lf_print_error_and_exit(\"Failed to get full_name from DESCRIPTOR.\");\n");
    code.append("}\n");
    code.append("const char* _lf_full_name = PyUnicode_AsUTF8(_lf_full_name_obj);\n");
    code.append("size_t _lf_name_len = strlen(_lf_full_name);\n");
    // Allocate and fill the wire buffer.
    code.append("size_t " + LEN_VAR + " = 4 + _lf_name_len + (size_t)_lf_proto_raw_len;\n");
    code.append("unsigned char* " + BUF_VAR + " = (unsigned char*)malloc(" + LEN_VAR + ");\n");
    code.append("if (" + BUF_VAR + " == NULL) {\n");
    code.append(
        "    lf_print_error_and_exit(\"Failed to allocate buffer for proto serialization.\");\n");
    code.append("}\n");
    code.append(BUF_VAR + "[0] = (unsigned char)((_lf_name_len >> 24) & 0xFF);\n");
    code.append(BUF_VAR + "[1] = (unsigned char)((_lf_name_len >> 16) & 0xFF);\n");
    code.append(BUF_VAR + "[2] = (unsigned char)((_lf_name_len >> 8) & 0xFF);\n");
    code.append(BUF_VAR + "[3] = (unsigned char)(_lf_name_len & 0xFF);\n");
    code.append("memcpy(" + BUF_VAR + " + 4, _lf_full_name, _lf_name_len);\n");
    code.append(
        "memcpy(" + BUF_VAR + " + 4 + _lf_name_len, _lf_proto_raw, (size_t)_lf_proto_raw_len);\n");
    code.append("Py_XDECREF(_lf_full_name_obj);\n");
    code.append("Py_XDECREF(_lf_proto_bytes);\n");
    return code;
  }

  /**
   * Generate C code that deserializes a wire buffer into a Python proto object.
   *
   * <p>The type name embedded in the wire format is used to look up the proto class via Python's
   * {@code google.protobuf.symbol_database}, so no port-type annotation is required.
   *
   * @param varName The action variable name (accessed as {@code varName->token->value}).
   * @param targetType Unused.
   */
  @Override
  public StringBuilder generateNetworkDeserializerCode(String varName, String targetType) {
    StringBuilder code = new StringBuilder();
    // Read wire format.
    code.append(
        "const unsigned char* _lf_wire = (const unsigned char*)" + varName + "->token->value;\n");
    code.append("size_t _lf_wire_len = (size_t)" + varName + "->token->length;\n");
    code.append("if (_lf_wire_len < 4) {\n");
    code.append("    lf_print_warning(\"Proto wire format too short. Dropping message.\");\n");
    code.append("    return;\n");
    code.append("}\n");
    code.append(
        "size_t _lf_full_name_len = ((size_t)_lf_wire[0] << 24) | ((size_t)_lf_wire[1] << 16)"
            + " | ((size_t)_lf_wire[2] << 8) | (size_t)_lf_wire[3];\n");
    code.append("if (_lf_wire_len < 4 + _lf_full_name_len) {\n");
    code.append(
        "    lf_print_warning(\"Proto wire format: name length exceeds buffer."
            + " Dropping message.\");\n");
    code.append("    return;\n");
    code.append("}\n");
    code.append("const unsigned char* _lf_proto_data = _lf_wire + 4 + _lf_full_name_len;\n");
    code.append("size_t _lf_proto_data_len = _lf_wire_len - 4 - _lf_full_name_len;\n");
    // Look up the proto class using Python's symbol database.
    code.append(
        "PyObject* _lf_sym_db_mod ="
            + " PyImport_ImportModule(\"google.protobuf.symbol_database\");\n");
    code.append("if (_lf_sym_db_mod == NULL) {\n");
    code.append("    if (PyErr_Occurred()) PyErr_Print();\n");
    code.append(
        "    lf_print_error_and_exit(\"Failed to import"
            + " google.protobuf.symbol_database.\");\n");
    code.append("}\n");
    code.append("PyObject* _lf_sym_db = PyObject_CallMethod(_lf_sym_db_mod, \"Default\", NULL);\n");
    code.append("Py_XDECREF(_lf_sym_db_mod);\n");
    code.append("if (_lf_sym_db == NULL) {\n");
    code.append("    if (PyErr_Occurred()) PyErr_Print();\n");
    code.append("    lf_print_error_and_exit(\"Failed to get symbol database.\");\n");
    code.append("}\n");
    code.append(
        "PyObject* _lf_full_name_pystr = PyUnicode_FromStringAndSize((const char*)(_lf_wire +"
            + " 4), _lf_full_name_len);\n");
    code.append(
        "PyObject* _lf_cls = PyObject_CallMethod(_lf_sym_db, \"GetSymbol\", \"O\","
            + " _lf_full_name_pystr);\n");
    code.append("Py_XDECREF(_lf_sym_db);\n");
    code.append("Py_XDECREF(_lf_full_name_pystr);\n");
    code.append("if (_lf_cls == NULL) {\n");
    code.append("    if (PyErr_Occurred()) PyErr_Print();\n");
    code.append(
        "    lf_print_warning(\"Failed to get proto class from symbol database."
            + " Dropping message.\");\n");
    code.append("    return;\n");
    code.append("}\n");
    // Create an instance and populate it from the wire bytes.
    code.append("PyObject* " + deserializedVarName + " = PyObject_CallNoArgs(_lf_cls);\n");
    code.append("Py_XDECREF(_lf_cls);\n");
    code.append("if (" + deserializedVarName + " == NULL) {\n");
    code.append("    if (PyErr_Occurred()) PyErr_Print();\n");
    code.append("    lf_print_warning(\"Failed to create proto instance. Dropping message.\");\n");
    code.append("    return;\n");
    code.append("}\n");
    code.append(
        "PyObject* _lf_proto_bytes_obj = PyBytes_FromStringAndSize((const char*)_lf_proto_data,"
            + " (Py_ssize_t)_lf_proto_data_len);\n");
    code.append(
        "PyObject* _lf_parse_result = PyObject_CallMethod("
            + deserializedVarName
            + ", \"ParseFromString\", \"O\", _lf_proto_bytes_obj);\n");
    code.append("Py_XDECREF(_lf_proto_bytes_obj);\n");
    code.append("if (_lf_parse_result == NULL) {\n");
    code.append("    if (PyErr_Occurred()) PyErr_Print();\n");
    code.append("    Py_XDECREF(" + deserializedVarName + ");\n");
    code.append(
        "    lf_print_warning(\"Failed to parse proto message from bytes."
            + " Dropping message.\");\n");
    code.append("    return;\n");
    code.append("}\n");
    code.append("Py_XDECREF(_lf_parse_result);\n");
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
