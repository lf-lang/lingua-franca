package org.lflang.federated.generator;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.DeadlineStats;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Connection;
import org.lflang.lf.Reaction;

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

  /**
   * Compute per-federate network listener deadlines without rebuilding the reaction graph.
   *
   * <p>Ordinary federate reactions keep inferred deadlines from the pre-proxy {@link
   * org.lflang.generator.ReactionInstanceGraph} (preserved by {@code clearCaches(false)}). Network
   * receiver deadlines on the AST (from {@link FedASTUtils}) inherit from local downstream
   * consumers of each port.
   *
   * <p>For each federate, sets {@link FederateInstance#rtiListenerDeadlineNs} to the minimum
   * inferred deadline among reactions in that federate, and {@link
   * FederateInstance#inboundP2PListenerDeadlineNs} to the minimum network-receiver AST deadline
   * among receivers fed by each inbound P2P peer.
   *
   * @param federationMain Federation reactor instance (pre-proxy graph; runtimes preserved).
   * @param federates All federate instances.
   */
  public static void computeNetworkListenerDeadlines(
      ReactorInstance federationMain, List<FederateInstance> federates) {
    for (FederateInstance federate : federates) {
      federate.rtiListenerDeadlineNs = Long.MAX_VALUE;
      federate.inboundP2PListenerDeadlineNs.clear();

      // Federate-wide min over ordinary reactions (inferred + level-tightened). Network
      // receiver AST deadlines inherit from these same local consumers, so they cannot be
      // tighter. Network sender deadlines may inherit from *remote* reactions and must not
      // define the RTI listener ("priority of this federate").
      ReactorInstance federateReactor =
          federationMain.getChildReactorInstance(federate.instantiation);
      TimeValue federateMin = minDeadlineInSubtree(federateReactor);

      if (federateMin != null && !TimeValue.isNoDeadlineSentinel(federateMin)) {
        federate.rtiListenerDeadlineNs = federateMin.toNanoSeconds();
      }

      for (FederateInstance peer : federate.inboundP2PConnections) {
        TimeValue peerMin = null;
        for (int i = 0; i < federate.networkReceiverReactions.size(); i++) {
          if (federate.networkMessageSourceFederate.get(i) != peer) {
            continue;
          }
          Reaction reaction = federate.networkReceiverReactions.get(i);
          TimeValue receiverDeadline =
              reaction.getDeadline() != null && reaction.getDeadline().getDelay() != null
                  ? ASTUtils.getLiteralTimeValue(reaction.getDeadline().getDelay())
                  : null;
          peerMin = minNullable(peerMin, receiverDeadline);
        }
        if (peerMin != null && !TimeValue.isNoDeadlineSentinel(peerMin)) {
          federate.inboundP2PListenerDeadlineNs.put(peer.id, peerMin.toNanoSeconds());
        }
      }
    }
  }

  /** Minimum non-sentinel inferred deadline in {@code ri} and its descendants. */
  private static TimeValue minDeadlineInSubtree(ReactorInstance ri) {
    if (ri == null) {
      return null;
    }
    TimeValue min = null;
    for (TimeValue deadline : DeadlineStats.collectAllDeadlines(ri)) {
      if (TimeValue.isNoDeadlineSentinel(deadline)) {
        continue;
      }
      min = minNullable(min, deadline);
    }
    return min;
  }

  /** Like {@link TimeValue#min}, but either argument may be null. */
  private static TimeValue minNullable(TimeValue a, TimeValue b) {
    if (a == null) {
      return b;
    }
    if (b == null) {
      return a;
    }
    return TimeValue.min(a, b);
  }
}
