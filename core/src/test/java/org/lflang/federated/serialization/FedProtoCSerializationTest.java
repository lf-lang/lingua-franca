package org.lflang.federated.serialization;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/** Tests for {@link FedProtoCSerialization}. */
public class FedProtoCSerializationTest {

  @Test
  public void protobufCFunctionPrefix() {
    assertEquals("person", FedProtoCSerialization.protobufCFunctionPrefix("Person"));
    assertEquals(
        "proto_hello_world", FedProtoCSerialization.protobufCFunctionPrefix("ProtoHelloWorld"));
  }
}
