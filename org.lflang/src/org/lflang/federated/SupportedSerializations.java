package org.lflang.federated;

/**
 * The supported serialization frameworks.
 * @author Soroush Bateni
 */
public enum SupportedSerializations {
    NATIVE("native"), // Dangerous: just copies the memory layout of the sender
    ROS2("ros2"),
    PROTO("proto");

    private String serialization;
    SupportedSerializations(String serialization) {
        this.serialization = serialization;
    }
    
    public String getSerialization() {
        return serialization;
    }
    
    public void setSerialization(String serialization) {
        this.serialization = serialization;
    }
}
