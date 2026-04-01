package org.lflang.federated.serialization;

/**
 * The supported serializers.
 *
 * @author Soroush Bateni
 * @ingroup Federated
 */
public enum SupportedSerializers {
  NATIVE("native"), // Dangerous: just copies the memory layout of the sender
  /**
   * Python federates: pickle over the network (same codegen as {@link FedNativePythonSerialization}).
   * Distinct from {@link #NATIVE} so the AST uses an opaque byte payload for {@code networkMessage}
   * actions instead of the port's C layout element size.
   */
  PICKLE("pickle"),
  ROS2("ros2"),
  PROTO("proto"),
  CUSTOM("");

  private String serializer;

  SupportedSerializers(String serializer) {
    this.serializer = serializer;
  }

  public String getSerializer() {
    return serializer;
  }

  public void setSerializer(String serializer) {
    this.serializer = serializer;
  }

  public static SupportedSerializers fromCustomString(String serializer) {
    CUSTOM.setSerializer(serializer);
    return CUSTOM;
  }
}
