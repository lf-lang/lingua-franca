package org.lflang.federated.serialization;

import org.lflang.generator.GeneratorBase;
import org.lflang.target.Target;

/**
 * Enables support for custom serialization.
 *
 * @author Shulu Li
 * @ingroup Federated
 */
public class FedCustomPythonSerialization implements FedSerialization {

  String customSerializerPackage;

  public FedCustomPythonSerialization(String customSerializerPackage) {
    this.customSerializerPackage = customSerializerPackage;
  }

  @Override
  public boolean isCompatible(GeneratorBase generator) {
    if (generator.getTarget() != Target.Python) {
      throw new UnsupportedOperationException(
          "The FedCustomPythonSerialization class only supports the Python target.");
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
    return "if (self->custom_serializer == NULL) \n"
        + "self->custom_serializer = load_serializer(\"%s\");\n".formatted(customSerializerPackage);
  }

  @Override
  public StringBuilder generateNetworkSerializerCode(String varName, String originalType) {
    StringBuilder serializerCode = new StringBuilder();
    // Initialize self->custom_serializer if null
    serializerCode.append(this.initializeCustomSerializer());
    // Serialize PyObject to bytes using custom serializer
    serializerCode.append(
        """
        PyObject *serialized_pyobject = custom_serialize(msg[0]->value, self->custom_serializer);
        Py_buffer %s;
        int returnValue = PyBytes_AsStringAndSize(serialized_pyobject, (char**)&%s.buf, &%s.len);
        if (returnValue == -1) {
            if (PyErr_Occurred()) PyErr_Print();
            lf_print_error_and_exit("Could not serialize %s.");
        }
        """
            .formatted(serializedVarName, serializedVarName, serializedVarName, serializedVarName));
    return serializerCode;
  }

  @Override
  public StringBuilder generateNetworkDeserializerCode(String varName, String targetType) {
    StringBuilder deserializerCode = new StringBuilder();
    // Initialize self->custom_serializer if null
    deserializerCode.append(this.initializeCustomSerializer());
    // Deserialize network message to a PyObject using custom serializer
    deserializerCode.append(
        """
PyObject *message_byte_array = PyBytes_FromStringAndSize((char*) %s->token->value, %s->token->length);
PyObject *%s = custom_deserialize(message_byte_array, self->custom_serializer);
if ( %s == NULL ) {
    if (PyErr_Occurred()) PyErr_Print();
    lf_print_error_and_exit("Could not serialize %s.");
}
"""
            .formatted(
                varName, varName, deserializedVarName, deserializedVarName, deserializedVarName));
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
