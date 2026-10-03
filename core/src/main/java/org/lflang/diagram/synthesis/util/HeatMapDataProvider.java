package org.lflang.diagram.synthesis.util;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Singleton that stores the latest reaction execution data received from a running Lingua Franca
 * program. Data is keyed by a composite string {@code "reactorName/reactionIndex"}.
 *
 * <p>Thread-safe: all internal state is stored in a {@link ConcurrentHashMap} and the global
 * min/max values are derived on read so that no locking is required.
 */
public final class HeatMapDataProvider {

  /** Execution statistics for a single reaction. */
  public static final class ReactionExecutionData {
    private final String reactor;
    private final int index;
    private final long avgNs;
    private final long maxNs;
    private final long count;

    public ReactionExecutionData(String reactor, int index, long avgNs, long maxNs, long count) {
      this.reactor = reactor;
      this.index = index;
      this.avgNs = avgNs;
      this.maxNs = maxNs;
      this.count = count;
    }

    public String getReactor() {
      return reactor;
    }

    public int getIndex() {
      return index;
    }

    public long getAvgNs() {
      return avgNs;
    }

    public long getMaxNs() {
      return maxNs;
    }

    public long getCount() {
      return count;
    }
  }

  private static final HeatMapDataProvider INSTANCE = new HeatMapDataProvider();

  private final ConcurrentHashMap<String, ReactionExecutionData> data = new ConcurrentHashMap<>();

  private HeatMapDataProvider() {}

  /** Returns the global singleton. */
  public static HeatMapDataProvider getInstance() {
    return INSTANCE;
  }

  /**
   * Build the composite key used for look-ups.
   *
   * @param reactorName the reactor instance name (as it appears in trace output)
   * @param reactionIndex zero-based reaction index
   * @return the key string
   */
  public static String key(String reactorName, int reactionIndex) {
    return reactorName + "/" + reactionIndex;
  }

  /**
   * Look up execution data for a specific reaction.
   *
   * @param reactorName the reactor instance name
   * @param reactionIndex zero-based reaction index
   * @return the data, or {@code null} if no data has been recorded yet
   */
  public ReactionExecutionData getExecutionData(String reactorName, int reactionIndex) {
    return data.get(key(reactorName, reactionIndex));
  }

  /**
   * Returns the minimum average execution time (in nanoseconds) across all tracked reactions, or
   * {@code 0} if no data is present.
   */
  public long getGlobalMinNanos() {
    return data.values().stream().mapToLong(ReactionExecutionData::getAvgNs).min().orElse(0);
  }

  /**
   * Returns the maximum average execution time (in nanoseconds) across all tracked reactions, or
   * {@code 0} if no data is present.
   */
  public long getGlobalMaxNanos() {
    return data.values().stream().mapToLong(ReactionExecutionData::getAvgNs).max().orElse(0);
  }

  /**
   * Replace the entire data set with {@code newData}. This is an atomic swap: any key that is not
   * present in {@code newData} will be removed.
   */
  public void updateData(Map<String, ReactionExecutionData> newData) {
    data.clear();
    data.putAll(newData);
  }

  /** Remove all stored data. */
  public void clear() {
    data.clear();
  }

  /** Returns {@code true} if execution data is available. */
  public boolean hasData() {
    return !data.isEmpty();
  }
}
