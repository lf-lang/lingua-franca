package org.lflang.federated.serialization;

/**
 * The supported serializers.
 *
 * @author Soroush Bateni
 */
public enum SupportedSerializers {
    NATIVE("native"), // Dangerous: just copies the memory layout of the sender
    ROS2("ros2"),
    PROTO("proto");

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
}
