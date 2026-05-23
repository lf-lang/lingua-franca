package org.lflang.federated.serialization;

import org.lflang.generator.GeneratorBase;
import org.lflang.target.Target;

/**
 * Enables support for Protocol Buffer serialization in C code using protobuf-c.
 *
 * <p>For federated connections that use {@code serializer "proto"}, the port type must be a
 * protobuf message <em>pointer</em> (e.g. {@code MyMessage*}). Sender reactions allocate and
 * initialize messages on the heap; the runtime owns the lifetime of deserialized messages on the
 * receiver side and frees them via {@link
 * org.lflang.federated.serialization.FedProtoCSerialization#PROTOBUF_DESTRUCTOR_NAME} (a thin
 * wrapper around {@code protobuf_c_message_free_unpacked}).
 *
 * @author Edward A. Lee
 * @ingroup Federated
 */
public class FedProtoCSerialization implements FedSerialization {

  /** Name of the generated destructor that frees a heap-allocated protobuf-c message. */
  public static final String PROTOBUF_DESTRUCTOR_NAME = "_lf_protobuf_destructor";

  /**
   * Convert a protobuf message type name to the protobuf-c function prefix. For example, {@code
   * Person} becomes {@code person} and {@code ProtoHelloWorld} becomes {@code proto_hello_world}.
   */
  public static String protobufCFunctionPrefix(String messageType) {
    StringBuilder prefix = new StringBuilder();
    for (int i = 0; i < messageType.length(); i++) {
      char c = messageType.charAt(i);
      if (Character.isUpperCase(c)) {
        if (i > 0) {
          prefix.append('_');
        }
        prefix.append(Character.toLowerCase(c));
      } else {
        prefix.append(c);
      }
    }
    return prefix.toString();
  }

  /** Strip trailing {@code *} characters from a C pointer type string. */
  public static String stripPointer(String type) {
    String result = type;
    while (result.endsWith("*")) {
      result = result.substring(0, result.length() - 1).trim();
    }
    return result;
  }

  @Override
  public boolean isCompatible(GeneratorBase generator) {
    if (generator.getTarget() != Target.C) {
      generator
          .messageReporter
          .nowhere()
          .error("Protobuf serialization is currently only supported for the C target.");
      return false;
    }
    return true;
  }

  @Override
  public String serializedBufferLength() {
    return serializedVarName + "_length";
  }

  @Override
  public String serializedBufferVar() {
    return serializedVarName;
  }

  /**
   * Generate code that serializes the value of a pointer-typed port using {@code __pack}.
   *
   * @param varName Reference to the port (e.g. {@code msg[0]}). The port's {@code value} field is
   *     expected to be a pointer to a protobuf message of type {@code originalType}.
   * @param originalType The protobuf message type name (without the trailing {@code *}).
   */
  @Override
  public StringBuilder generateNetworkSerializerCode(String varName, String originalType) {
    String prefix = protobufCFunctionPrefix(stripPointer(originalType));
    String valueRef = varName + "->value";
    StringBuilder code = new StringBuilder();
    code.append(
        "size_t "
            + serializedVarName
            + "_length = "
            + prefix
            + "__get_packed_size("
            + valueRef
            + ");\n");
    code.append(
        "unsigned char* "
            + serializedVarName
            + " = (unsigned char*)malloc("
            + serializedVarName
            + "_length);\n");
    code.append("if (" + serializedVarName + " == NULL) {\n");
    code.append(
        "    lf_print_error_and_exit(\"Failed to allocate buffer for protobuf serialization.\");\n");
    code.append("}\n");
    code.append(prefix + "__pack(" + valueRef + ", " + serializedVarName + ");\n");
    return code;
  }

  /**
   * Generate code that deserializes a network message into a freshly heap-allocated protobuf
   * message via {@code __unpack}.
   *
   * @param varName Reference to the receiving action.
   * @param targetType The protobuf message type name (without the trailing {@code *}).
   */
  @Override
  public StringBuilder generateNetworkDeserializerCode(String varName, String targetType) {
    String base = stripPointer(targetType);
    String prefix = protobufCFunctionPrefix(base);
    StringBuilder code = new StringBuilder();
    code.append(
        base
            + " *"
            + deserializedVarName
            + " = "
            + prefix
            + "__unpack(NULL, "
            + varName
            + ".tmplt.token->length, (uint8_t*)"
            + varName
            + ".tmplt.token->value);\n");
    code.append("if (" + deserializedVarName + " == NULL) {\n");
    code.append(
        "    lf_print_error_and_exit(\"Could not deserialize protobuf message.\");\n");
    code.append("}\n");
    return code;
  }

  /**
   * Generate code that hands ownership of the deserialized message to the LF runtime via a token,
   * so that {@code protobuf_c_message_free_unpacked} is invoked when the token's reference count
   * drops to zero.
   */
  public StringBuilder generatePortAssignmentCode(String receiveRef) {
    StringBuilder code = new StringBuilder();
    code.append(
        "lf_token_t* _lf_proto_token = lf_new_token((void*)"
            + receiveRef
            + ", "
            + deserializedVarName
            + ", 1);\n");
    code.append(
        "lf_set_destructor(" + receiveRef + ", " + PROTOBUF_DESTRUCTOR_NAME + ");\n");
    code.append("lf_set_token(" + receiveRef + ", _lf_proto_token);\n");
    return code;
  }

  @Override
  public StringBuilder generatePreambleForSupport() {
    StringBuilder preamble = new StringBuilder();
    preamble.append("#include <protobuf-c/protobuf-c.h>\n");
    preamble.append(
        """
        static void %s(void* value) {
          if (value != NULL) {
            protobuf_c_message_free_unpacked((ProtobufCMessage*)value, NULL);
          }
        }
        """
            .formatted(PROTOBUF_DESTRUCTOR_NAME));
    return preamble;
  }

  @Override
  public StringBuilder generateCompilerExtensionForSupport() {
    return new StringBuilder();
  }
}
