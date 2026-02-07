package org.lflang.federated.generator;

import com.google.gson.JsonObject;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.lflang.MessageReporter;
import org.lflang.TimeUnit;
import org.lflang.TimeValue;
import org.lflang.federated.serialization.SupportedSerializers;
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
    // Collect all deadlines from the federation
    List<TimeValue> allDeadlines = collectAllDeadlinesFromFederation(federationMain);

    // Filter out sentinel values (NEVER, MAX_VALUE, FOREVER indicate no deadline)
    // Use .equals() for value comparison since TimeValue overrides equals()
    List<TimeValue> validDeadlines = allDeadlines.stream()
        .filter(d -> !TimeValue.NEVER.equals(d) && !TimeValue.MAX_VALUE.equals(d) && !TimeValue.FOREVER.equals(d))
        .sorted()
        .collect(Collectors.toList());

    // Create JSON object with deadline statistics
    JsonObject json = new JsonObject();
    
    if (validDeadlines.isEmpty()) {
      // No valid deadlines found in the federation
      json.addProperty("minDeadlineMs", 0.0);
      json.addProperty("maxDeadlineMs", 0.0);
      json.addProperty("medianDeadlineMs", 0.0);
      json.addProperty("totalDeadlines", 0);
    } else {
      // Compute statistics
      TimeValue minDeadline = validDeadlines.get(0);
      TimeValue maxDeadline = validDeadlines.get(validDeadlines.size() - 1);
      TimeValue medianDeadline;
      if (validDeadlines.size() % 2 == 0) {
        // Even number: median is average of two middle values
        int mid = validDeadlines.size() / 2;
        long medianNanos = (validDeadlines.get(mid - 1).toNanoSeconds() 
            + validDeadlines.get(mid).toNanoSeconds()) / 2;
        medianDeadline = new TimeValue(medianNanos, TimeUnit.NANO);
      } else {
        // Odd number: median is the middle value
        medianDeadline = validDeadlines.get(validDeadlines.size() / 2);
      }

      // Convert to milliseconds
      double minDeadlineMs = minDeadline.toNanoSeconds() / 1_000_000.0;
      double maxDeadlineMs = maxDeadline.toNanoSeconds() / 1_000_000.0;
      double medianDeadlineMs = medianDeadline.toNanoSeconds() / 1_000_000.0;

      json.addProperty("minDeadlineMs", minDeadlineMs);
      json.addProperty("maxDeadlineMs", maxDeadlineMs);
      json.addProperty("medianDeadlineMs", medianDeadlineMs);
      json.addProperty("totalDeadlines", validDeadlines.size());
    }

    // Write the JSON file
    String relPath = "include" + File.separator + "federation_properties.json";
    Path jsonPath = fileConfig.getSrcPath().resolve(relPath);
    Files.createDirectories(jsonPath.getParent());
    try (var writer = Files.newBufferedWriter(jsonPath, StandardCharsets.UTF_8)) {
      writer.write(json.toString());
    }
  }

  /**
   * Recursively collect all inferred deadlines from a federation ReactorInstance.
   * This includes deadlines from all federates and their nested reactors.
   * 
   * Note: This uses inferred deadlines which include deadline propagation through
   * the reaction graph - if a downstream reaction has an earlier deadline, it is
   * propagated to upstream reactions.
   * 
   * @param instance The federation ReactorInstance.
   * @return A list of all inferred deadlines found in the federation.
   */
  private static List<TimeValue> collectAllDeadlinesFromFederation(ReactorInstance instance) {
    List<TimeValue> deadlines = new ArrayList<>();

    // Collect inferred deadlines from all reactions in this instance
    for (var reaction : instance.reactions) {
      deadlines.addAll(reaction.getInferredDeadlinesList());
    }

    // Recursively collect deadlines from child reactor instances
    for (ReactorInstance child : instance.children) {
      deadlines.addAll(collectAllDeadlinesFromFederation(child));
    }

    return deadlines;
  }
}
