package org.lflang.federated.generator;

import java.io.IOException;
import java.nio.file.Path;
import org.lflang.MessageReporter;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.DeadlineStats;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Connection;

/**
 * A collection of utility methods for the federated generator.
 *
 * @ingroup Federated
 */
public class FedUtils {
  /**
   * Get the serializer for the `connection` between `srcFederate` and `dstFederate`.
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

  /**
   * Generate a JSON file with federation-level deadline statistics.
   * This is called once from FedGenerator for the entire federation.
   *
   * @param fileConfig The federation file configuration.
   * @param federationMain The ReactorInstance representing the entire federation.
   * @param messageReporter Used to report errors.
   * @throws IOException If file writing fails.
   */
  public static void generateFederationPropertiesFile(
      FederationFileConfig fileConfig,
      ReactorInstance federationMain,
      MessageReporter messageReporter)
      throws IOException {
    DeadlineStats stats = DeadlineStats.fromReactorInstance(federationMain);
    Path jsonPath = fileConfig.getSrcPath().resolve(DeadlineStats.FEDERATION_PROPERTIES_REL_PATH);
    stats.writeJson(jsonPath);
  }
}
