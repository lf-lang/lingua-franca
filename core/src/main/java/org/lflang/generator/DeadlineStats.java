package org.lflang.generator;

import com.google.gson.JsonArray;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import java.io.File;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.lflang.TimeUnit;
import org.lflang.TimeValue;

/**
 * Deadline statistics derived from a {@link ReactorInstance} tree.
 *
 * <p>Used both for non-federated priority-function generation and for writing/reading the
 * federation-level {@code federation_properties.json} file that federates consult so every
 * federate shares the same deadline-to-priority mapping.
 *
 * @ingroup Instances
 */
public class DeadlineStats {

  /** Relative path under a federation's {@code src} directory for the properties JSON file. */
  public static final String FEDERATION_PROPERTIES_REL_PATH =
      "include" + File.separator + "federation_properties.json";

  private final double minDeadlineMs;
  private final double maxDeadlineMs;
  private final double medianDeadlineMs;
  private final List<Double> deadlinesMs;
  private final List<Double> distinctDeadlinesMs;

  private DeadlineStats(
      double minDeadlineMs,
      double maxDeadlineMs,
      double medianDeadlineMs,
      List<Double> deadlinesMs) {
    this.minDeadlineMs = minDeadlineMs;
    this.maxDeadlineMs = maxDeadlineMs;
    this.medianDeadlineMs = medianDeadlineMs;
    this.deadlinesMs = List.copyOf(deadlinesMs);
    this.distinctDeadlinesMs =
        deadlinesMs.stream().distinct().sorted().collect(Collectors.toUnmodifiableList());
  }

  /** Empty / no-deadline statistics (all zeros). */
  public static DeadlineStats empty() {
    return new DeadlineStats(0.0, 0.0, 0.0, List.of());
  }

  /**
   * Collect inferred deadlines from {@code root} and its descendants, drop sentinel "no deadline"
   * values, and compute statistics.
   *
   * <p>Inferred deadlines include deadline propagation through the reaction graph — if a
   * downstream reaction has an earlier deadline, it is propagated to upstream reactions.
   */
  public static DeadlineStats fromReactorInstance(ReactorInstance root) {
    List<TimeValue> validDeadlines =
        collectAllDeadlines(root).stream()
            .filter(
                d ->
                    !TimeValue.NEVER.equals(d)
                        && !TimeValue.MAX_VALUE.equals(d)
                        && !TimeValue.FOREVER.equals(d))
            .sorted()
            .collect(Collectors.toList());

    if (validDeadlines.isEmpty()) {
      return empty();
    }

    TimeValue min = validDeadlines.get(0);
    TimeValue max = validDeadlines.get(validDeadlines.size() - 1);
    TimeValue median;
    if (validDeadlines.size() % 2 == 0) {
      int mid = validDeadlines.size() / 2;
      long medianNanos =
          (validDeadlines.get(mid - 1).toNanoSeconds() + validDeadlines.get(mid).toNanoSeconds())
              / 2;
      median = new TimeValue(medianNanos, TimeUnit.NANO);
    } else {
      median = validDeadlines.get(validDeadlines.size() / 2);
    }

    List<Double> deadlinesMs = new ArrayList<>(validDeadlines.size());
    for (TimeValue d : validDeadlines) {
      deadlinesMs.add(d.toNanoSeconds() / 1_000_000.0);
    }

    return new DeadlineStats(
        min.toNanoSeconds() / 1_000_000.0,
        max.toNanoSeconds() / 1_000_000.0,
        median.toNanoSeconds() / 1_000_000.0,
        deadlinesMs);
  }

  /** Whether there are no finite deadlines, i.e. every statistic is zero. */
  public boolean isEmpty() {
    return minDeadlineMs == 0.0 && maxDeadlineMs == 0.0 && medianDeadlineMs == 0.0;
  }

  public double getMinDeadlineMs() {
    return minDeadlineMs;
  }

  public double getMaxDeadlineMs() {
    return maxDeadlineMs;
  }

  public double getMedianDeadlineMs() {
    return medianDeadlineMs;
  }

  /** All valid deadlines in milliseconds (sorted, may contain duplicates). */
  public List<Double> getDeadlinesMs() {
    return deadlinesMs;
  }

  /** Distinct valid deadlines in milliseconds (sorted). */
  public List<Double> getDistinctDeadlinesMs() {
    return distinctDeadlinesMs;
  }

  /** Serialize these statistics to the federation-properties JSON format. */
  public JsonObject toJson() {
    JsonObject json = new JsonObject();
    json.addProperty("minDeadlineMs", minDeadlineMs);
    json.addProperty("maxDeadlineMs", maxDeadlineMs);
    json.addProperty("medianDeadlineMs", medianDeadlineMs);
    json.addProperty("totalDeadlines", deadlinesMs.size());
    JsonArray arr = new JsonArray();
    for (double d : deadlinesMs) {
      arr.add(d);
    }
    json.add("deadlinesMs", arr);
    return json;
  }

  /** Write these statistics to {@code path} as JSON. */
  public void writeJson(Path path) throws IOException {
    Files.createDirectories(path.getParent());
    try (var writer = Files.newBufferedWriter(path, StandardCharsets.UTF_8)) {
      writer.write(toJson().toString());
    }
  }

  /**
   * Parse statistics from a federation-properties JSON file.
   *
   * @throws IOException If the file cannot be read.
   * @throws RuntimeException If the JSON is malformed or missing required fields.
   */
  public static DeadlineStats readJson(Path path) throws IOException {
    String jsonContent = Files.readString(path, StandardCharsets.UTF_8);
    JsonObject json = JsonParser.parseString(jsonContent).getAsJsonObject();
    double min = json.get("minDeadlineMs").getAsDouble();
    double max = json.get("maxDeadlineMs").getAsDouble();
    double median = json.get("medianDeadlineMs").getAsDouble();
    List<Double> deadlinesMs = new ArrayList<>();
    if (json.has("deadlinesMs") && json.get("deadlinesMs").isJsonArray()) {
      JsonArray arr = json.getAsJsonArray("deadlinesMs");
      for (var el : arr) {
        if (el != null && el.isJsonPrimitive() && el.getAsJsonPrimitive().isNumber()) {
          deadlinesMs.add(el.getAsDouble());
        }
      }
    }
    return new DeadlineStats(min, max, median, deadlinesMs);
  }

  /**
   * Recursively collect all inferred deadlines from a reactor instance tree.
   *
   * @param instance The reactor instance to collect deadlines from.
   * @return A list of all deadlines found in this instance and its children.
   */
  public static List<TimeValue> collectAllDeadlines(ReactorInstance instance) {
    List<TimeValue> deadlines = new ArrayList<>();
    for (ReactionInstance reaction : instance.reactions) {
      deadlines.addAll(reaction.getInferredDeadlinesList());
    }
    for (ReactorInstance child : instance.children) {
      deadlines.addAll(collectAllDeadlines(child));
    }
    return deadlines;
  }
}
