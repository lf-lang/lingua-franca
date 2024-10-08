package org.lflang.federated.generator;

import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.lf.Connection;

/** A collection of utility methods for the federated generator. */
public class FedUtils {
  /**
   * Get the serializer for the {@code connection} between {@code srcFederate} and {@code
   * dstFederate}.
   */
  public static SupportedSerializers getSerializer(
      Connection connection, FederateInstance srcFederate, FederateInstance dstFederate) {
    // Get the serializer
    SupportedSerializers serializer = SupportedSerializers.NATIVE;
    if (connection.getSerializer() != null) {
      boolean isCustomSerializer = true;
      for (SupportedSerializers method : SupportedSerializers.values()) {
        if (method.name().equalsIgnoreCase(connection.getSerializer().getType())) {
          serializer =
              SupportedSerializers.valueOf(connection.getSerializer().getType().toUpperCase());
          isCustomSerializer = false;
          break;
        }
      }
      if (isCustomSerializer) {
        serializer = SupportedSerializers.fromCustomString(connection.getSerializer().getType());
      }
    }
    // Add it to the list of enabled serializers for the source and destination federates
    srcFederate.enabledSerializers.add(serializer);
    dstFederate.enabledSerializers.add(serializer);
    return serializer;
  }
}
