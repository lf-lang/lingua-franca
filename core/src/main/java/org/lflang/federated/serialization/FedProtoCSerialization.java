package org.lflang.federated.serialization;

import org.lflang.generator.GeneratorBase;
import org.lflang.target.Target;

/**
 * Enables support for Protocol Buffer serialization in C code using protobuf-c.
 *
 * @author Edward A. Lee
 * @ingroup Federated
 */
public class FedProtoCSerialization implements FedSerialization {

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

  @Override
  public StringBuilder generateNetworkSerializerCode(String varName, String originalType) {
    String prefix = protobufCFunctionPrefix(originalType);
    String valueRef = varName + "->value";
    StringBuilder serializerCode = new StringBuilder();
    serializerCode.append(
        "if (((const ProtobufCMessage *)&"
            + valueRef
            + ")->descriptor == NULL) {\n");
    serializerCode.append(
        "    ProtobufCMessage *_lf_msg_base = (ProtobufCMessage *)&" + valueRef + ";\n");
    serializerCode.append(
        "    _lf_msg_base->descriptor = &" + prefix + "__descriptor;\n");
    serializerCode.append("    _lf_msg_base->n_unknown_fields = 0;\n");
    serializerCode.append("    _lf_msg_base->unknown_fields = NULL;\n");
    serializerCode.append("}\n");
    serializerCode.append(
        "size_t "
            + serializedVarName
            + "_length = "
            + prefix
            + "__get_packed_size(&"
            + valueRef
            + ");\n");
    serializerCode.append(
        "unsigned char* "
            + serializedVarName
            + " = (unsigned char*)malloc("
            + serializedVarName
            + "_length);\n");
    serializerCode.append("if (" + serializedVarName + " == NULL) {\n");
    serializerCode.append(
        "    lf_print_error_and_exit(\"Failed to allocate buffer for protobuf serialization.\");\n");
    serializerCode.append("}\n");
    serializerCode.append(prefix + "__pack(&" + valueRef + ", " + serializedVarName + ");\n");
    return serializerCode;
  }

  @Override
  public StringBuilder generateNetworkDeserializerCode(String varName, String targetType) {
    String prefix = protobufCFunctionPrefix(targetType);
    StringBuilder deserializerCode = new StringBuilder();
    deserializerCode.append(
        targetType + " *" + deserializedVarName + " = " + prefix + "__unpack(NULL, ");
    deserializerCode.append(varName + ".tmplt.token->length, ");
    deserializerCode.append("(uint8_t*)" + varName + ".tmplt.token->value);\n");
    deserializerCode.append("if (" + deserializedVarName + " == NULL) {\n");
    deserializerCode.append(
        "    lf_print_error_and_exit(\"Could not deserialize protobuf message.\");\n");
    deserializerCode.append("}\n");
    return deserializerCode;
  }

  /**
   * Generate code that copies a deserialized protobuf message into a port value.
   *
   * @param receiveRef A target language reference to the receiving port.
   */
  public StringBuilder generatePortAssignmentCode(String receiveRef) {
    StringBuilder code = new StringBuilder();
    code.append(
        "lf_assign_protobuf_message(&"
            + receiveRef
            + "->value, (ProtobufCMessage*)"
            + deserializedVarName
            + ");\n");
    code.append("lf_set_present(" + receiveRef + ");\n");
    return code;
  }

  @Override
  public StringBuilder generatePreambleForSupport() {
    StringBuilder preamble = new StringBuilder();
    preamble.append("#include <protobuf-c/protobuf-c.h>\n");
    preamble.append("#include <string.h>\n");
    preamble.append("#include <stdlib.h>\n");
    preamble.append(
        """
        // Free heap-owned fields inside a stack-allocated protobuf message.
        // Unlike protobuf_c_message_free_unpacked(), this does not free the message struct itself.
        static void lf_free_protobuf_message_fields(ProtobufCMessage *message) {
          const ProtobufCMessageDescriptor *desc;
          unsigned f;

          if (message == NULL || message->descriptor == NULL) {
            return;
          }
          desc = message->descriptor;
          for (f = 0; f < desc->n_fields; f++) {
            if (0 != (desc->fields[f].flags & PROTOBUF_C_FIELD_FLAG_ONEOF) &&
                desc->fields[f].id !=
                    *(uint32_t *)((char *)message + desc->fields[f].quantifier_offset)) {
              continue;
            }
            if (desc->fields[f].label == PROTOBUF_C_LABEL_REPEATED) {
              size_t n = *(size_t *)((char *)message + desc->fields[f].quantifier_offset);
              void *arr = *(void **)((char *)message + desc->fields[f].offset);
              if (arr != NULL) {
                if (desc->fields[f].type == PROTOBUF_C_TYPE_STRING) {
                  unsigned i;
                  for (i = 0; i < n; i++) {
                    free(((char **)arr)[i]);
                  }
                } else if (desc->fields[f].type == PROTOBUF_C_TYPE_BYTES) {
                  unsigned i;
                  for (i = 0; i < n; i++) {
                    free(((ProtobufCBinaryData *)arr)[i].data);
                  }
                } else if (desc->fields[f].type == PROTOBUF_C_TYPE_MESSAGE) {
                  unsigned i;
                  for (i = 0; i < n; i++) {
                    protobuf_c_message_free_unpacked(((ProtobufCMessage **)arr)[i], NULL);
                  }
                }
                free(arr);
              }
            } else if (desc->fields[f].type == PROTOBUF_C_TYPE_STRING) {
              char *str = *(char **)((char *)message + desc->fields[f].offset);
              if (str && str != desc->fields[f].default_value) {
                free(str);
              }
            } else if (desc->fields[f].type == PROTOBUF_C_TYPE_BYTES) {
              ProtobufCBinaryData *bd = (ProtobufCBinaryData *)((char *)message + desc->fields[f].offset);
              const ProtobufCBinaryData *default_bd = desc->fields[f].default_value;
              if (bd->data != NULL && (default_bd == NULL || default_bd->data != bd->data)) {
                free(bd->data);
              }
            } else if (desc->fields[f].type == PROTOBUF_C_TYPE_MESSAGE) {
              ProtobufCMessage *sm = *(ProtobufCMessage **)((char *)message + desc->fields[f].offset);
              if (sm && sm != desc->fields[f].default_value) {
                protobuf_c_message_free_unpacked(sm, NULL);
              }
            }
          }
          for (f = 0; f < message->n_unknown_fields; f++) {
            free(message->unknown_fields[f].data);
          }
          if (message->unknown_fields != NULL) {
            free(message->unknown_fields);
          }
          protobuf_c_message_init(desc, message);
        }

        static void lf_deep_copy_protobuf_message(ProtobufCMessage *dest, const ProtobufCMessage *src) {
          const ProtobufCMessageDescriptor *desc = src->descriptor;
          unsigned f;

          for (f = 0; f < desc->n_fields; f++) {
            const ProtobufCFieldDescriptor *field = &desc->fields[f];
            void *dest_field = (char *)dest + field->offset;
            const void *src_field = (const char *)src + field->offset;

            if (0 != (field->flags & PROTOBUF_C_FIELD_FLAG_ONEOF) &&
                field->id != *(uint32_t *)((char *)src + field->quantifier_offset)) {
              continue;
            }

            if (field->label == PROTOBUF_C_LABEL_REPEATED) {
              size_t n = *(size_t *)((char *)src + field->quantifier_offset);
              *(size_t *)((char *)dest + field->quantifier_offset) = n;
              if (n == 0) {
                *(void **)dest_field = NULL;
                continue;
              }
              if (field->type == PROTOBUF_C_TYPE_STRING) {
                char **src_arr = *(char ***)src_field;
                char **dest_arr = (char **)malloc(n * sizeof(char *));
                if (dest_arr == NULL) {
                  lf_print_error_and_exit("Failed to allocate buffer for protobuf deserialization.");
                }
                for (size_t i = 0; i < n; i++) {
                  dest_arr[i] = src_arr[i] ? strdup(src_arr[i]) : NULL;
                }
                *(char ***)dest_field = dest_arr;
              } else if (field->type == PROTOBUF_C_TYPE_MESSAGE) {
                lf_print_error_and_exit("Repeated protobuf sub-messages are not supported yet.");
              } else {
                size_t element_size = 0;
                switch (field->type) {
                  case PROTOBUF_C_TYPE_INT32:
                  case PROTOBUF_C_TYPE_SINT32:
                  case PROTOBUF_C_TYPE_SFIXED32:
                  case PROTOBUF_C_TYPE_UINT32:
                  case PROTOBUF_C_TYPE_FIXED32:
                  case PROTOBUF_C_TYPE_FLOAT:
                  case PROTOBUF_C_TYPE_ENUM:
                    element_size = 4;
                    break;
                  case PROTOBUF_C_TYPE_INT64:
                  case PROTOBUF_C_TYPE_SINT64:
                  case PROTOBUF_C_TYPE_SFIXED64:
                  case PROTOBUF_C_TYPE_UINT64:
                  case PROTOBUF_C_TYPE_FIXED64:
                  case PROTOBUF_C_TYPE_DOUBLE:
                    element_size = 8;
                    break;
                  case PROTOBUF_C_TYPE_BOOL:
                    element_size = sizeof(protobuf_c_boolean);
                    break;
                  default:
                    lf_print_error_and_exit("Unsupported repeated protobuf field type.");
                }
                void *dest_arr = malloc(n * element_size);
                if (dest_arr == NULL) {
                  lf_print_error_and_exit("Failed to allocate buffer for protobuf deserialization.");
                }
                memcpy(dest_arr, *(void **)src_field, n * element_size);
                *(void **)dest_field = dest_arr;
              }
            } else {
              switch (field->type) {
                case PROTOBUF_C_TYPE_STRING: {
                  const char *src_str = *(const char **)src_field;
                  *(char **)dest_field = src_str ? strdup(src_str) : NULL;
                  break;
                }
                case PROTOBUF_C_TYPE_BYTES: {
                  const ProtobufCBinaryData *src_bd = (const ProtobufCBinaryData *)src_field;
                  ProtobufCBinaryData *dest_bd = (ProtobufCBinaryData *)dest_field;
                  dest_bd->len = src_bd->len;
                  if (src_bd->len > 0) {
                    dest_bd->data = (uint8_t *)malloc(src_bd->len);
                    if (dest_bd->data == NULL) {
                      lf_print_error_and_exit("Failed to allocate buffer for protobuf deserialization.");
                    }
                    memcpy(dest_bd->data, src_bd->data, src_bd->len);
                  } else {
                    dest_bd->data = NULL;
                  }
                  break;
                }
                case PROTOBUF_C_TYPE_MESSAGE:
                  lf_print_error_and_exit("Nested protobuf sub-messages are not supported yet.");
                  break;
                case PROTOBUF_C_TYPE_INT32:
                case PROTOBUF_C_TYPE_SINT32:
                case PROTOBUF_C_TYPE_SFIXED32:
                case PROTOBUF_C_TYPE_UINT32:
                case PROTOBUF_C_TYPE_FIXED32:
                case PROTOBUF_C_TYPE_FLOAT:
                case PROTOBUF_C_TYPE_ENUM:
                  *(uint32_t *)dest_field = *(const uint32_t *)src_field;
                  break;
                case PROTOBUF_C_TYPE_INT64:
                case PROTOBUF_C_TYPE_SINT64:
                case PROTOBUF_C_TYPE_SFIXED64:
                case PROTOBUF_C_TYPE_UINT64:
                case PROTOBUF_C_TYPE_FIXED64:
                case PROTOBUF_C_TYPE_DOUBLE:
                  *(uint64_t *)dest_field = *(const uint64_t *)src_field;
                  break;
                case PROTOBUF_C_TYPE_BOOL:
                  *(protobuf_c_boolean *)dest_field = *(const protobuf_c_boolean *)src_field;
                  break;
                default:
                  lf_print_error_and_exit("Unsupported protobuf field type.");
                  break;
              }
            }
          }
        }

        static void lf_assign_protobuf_message(void *dest, ProtobufCMessage *src) {
          lf_free_protobuf_message_fields((ProtobufCMessage *)dest);
          lf_deep_copy_protobuf_message((ProtobufCMessage *)dest, src);
          protobuf_c_message_free_unpacked(src, NULL);
        }
        """);
    return preamble;
  }

  @Override
  public StringBuilder generateCompilerExtensionForSupport() {
    return new StringBuilder();
  }
}
