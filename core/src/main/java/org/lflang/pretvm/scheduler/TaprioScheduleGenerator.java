package org.lflang.pretvm.scheduler;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import org.lflang.TimeValue;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CFileConfig;
import org.lflang.pretvm.dag.Dag;
import org.lflang.pretvm.dag.DagEdge;
import org.lflang.pretvm.dag.DagNode;
import org.lflang.pretvm.dag.IntervalNode;
import org.lflang.pretvm.dag.JobNode;

/**
 * Generates a TAPRIO qdisc configuration script from an unpartitioned DAG.
 *
 * <p>Given a DAG for a periodic phase, this class identifies inter-federate (top-level reactor)
 * communication edges, computes the worst-case makespan up to each sending reaction, and generates
 * a bash script that configures the Linux TAPRIO qdisc accordingly.
 *
 * @author Shaokai J. Lin
 */
public class TaprioScheduleGenerator {

  /** The main reactor instance (depth 0). */
  private final ReactorInstance main;

  /** The output directory for the generated script. */
  private final Path outputDir;

  /**
   * Constructor.
   *
   * @param main the main reactor instance
   * @param fileConfig the C file configuration
   */
  public TaprioScheduleGenerator(ReactorInstance main, CFileConfig fileConfig) {
    this(main, fileConfig.getSrcGenPath());
  }

  /**
   * Constructor with explicit output directory.
   *
   * @param main the main reactor instance
   * @param outputDir the directory in which to write the generated script
   */
  public TaprioScheduleGenerator(ReactorInstance main, Path outputDir) {
    this.main = main;
    this.outputDir = outputDir;
  }

  /**
   * Generate a TAPRIO qdisc configuration script from the given unpartitioned DAG.
   *
   * @param dag the unpartitioned DAG for a periodic phase
   * @param hyperperiodNs the hyperperiod in nanoseconds
   * @param phaseIndex the index of the periodic phase
   */
  public void generate(Dag dag, long hyperperiodNs, int phaseIndex) {
    // Find inter-federate edges.
    List<DagEdge> interFederateEdges = findInterFederateEdges(dag);

    // Skip silently if no inter-federate communication exists.
    if (interFederateEdges.isEmpty()) {
      return;
    }

    // Compute makespans from DAG start to every node.
    Map<DagNode, Long> makespans = computeMakespans(dag);
    if (makespans == null) {
      // A warning was logged due to unannotated WCET; skip TAPRIO generation.
      return;
    }

    // Build traffic class map: each depth-1 reactor gets a TC index.
    Map<ReactorInstance, Integer> tcMap = buildTrafficClassMap();

    // For each sending federate, find the max makespan across all its inter-federate sends.
    Map<ReactorInstance, Long> sendMakespans = new LinkedHashMap<>();
    for (DagEdge edge : interFederateEdges) {
      JobNode sourceJob = (JobNode) edge.sourceNode;
      ReactorInstance sourceFederate = getTopLevelReactor(sourceJob.getReaction());
      long makespan = makespans.getOrDefault(sourceJob, 0L);
      sendMakespans.merge(sourceFederate, makespan, Math::max);
    }

    // Generate the script.
    String script = generateScript(tcMap, sendMakespans, hyperperiodNs);

    // Write the script to the output directory.
    Path outputPath = outputDir.resolve("taprio_setup.sh");
    try {
      Files.writeString(outputPath, script);
      outputPath.toFile().setExecutable(true);
    } catch (IOException e) {
      throw new RuntimeException("Failed to write TAPRIO script: " + e.getMessage(), e);
    }
  }

  /**
   * Compute the longest-path makespan from the DAG start to every node using topological order.
   *
   * @param dag the DAG
   * @return a map from each node to its makespan in nanoseconds, or null if any JobNode on a path
   *     to a send has unannotated WCET
   */
  private Map<DagNode, Long> computeMakespans(Dag dag) {
    List<DagNode> topoOrder = dag.getTopologicalSort();
    Map<DagNode, Long> makespan = new HashMap<>();

    for (DagNode node : topoOrder) {
      // Compute the cost of this node.
      long nodeCost = 0;
      if (node instanceof JobNode jobNode) {
        if (jobNode.getReaction().wcet.equals(TimeValue.MAX_VALUE)) {
          System.err.println(
              "WARNING: Reaction "
                  + jobNode.getReaction().getFullName()
                  + " has unannotated WCET. Skipping TAPRIO generation.");
          return null;
        }
        nodeCost = jobNode.getReaction().wcet.toNanoSeconds();
      } else if (node instanceof IntervalNode intervalNode) {
        nodeCost = intervalNode.getInterval().toNanoSeconds();
      }
      // TimeNode has cost 0.

      // Find max makespan among all upstream nodes.
      long maxUpstream = 0;
      for (DagNode pred : dag.getUpstreamNodes(node)) {
        maxUpstream = Math.max(maxUpstream, makespan.getOrDefault(pred, 0L));
      }

      makespan.put(node, maxUpstream + nodeCost);
    }

    return makespan;
  }

  /**
   * Find DAG edges where the source and sink JobNodes belong to different depth-1 reactors.
   *
   * @param dag the DAG
   * @return a list of inter-federate edges
   */
  private List<DagEdge> findInterFederateEdges(Dag dag) {
    List<DagEdge> result = new ArrayList<>();
    for (DagEdge edge : dag.getDagEdges()) {
      if (edge.sourceNode instanceof JobNode sourceJob
          && edge.sinkNode instanceof JobNode sinkJob) {
        ReactorInstance sourceReactor = getTopLevelReactor(sourceJob.getReaction());
        ReactorInstance sinkReactor = getTopLevelReactor(sinkJob.getReaction());
        if (sourceReactor != null && sinkReactor != null && sourceReactor != sinkReactor) {
          result.add(edge);
        }
      }
    }
    return result;
  }

  /**
   * Resolve a ReactionInstance to its depth-1 ancestor ReactorInstance.
   *
   * @param reaction the reaction instance
   * @return the depth-1 reactor instance, or null if not found
   */
  private ReactorInstance getTopLevelReactor(ReactionInstance reaction) {
    ReactorInstance reactor = reaction.getParent();
    if (reactor == null) return null;
    if (reactor.getDepth() == 1) return reactor;
    // Walk up to the depth-1 ancestor.
    return reaction.getParent(1);
  }

  /**
   * Assign a traffic class index to each depth-1 reactor (direct child of main).
   *
   * @return a map from each depth-1 reactor to its traffic class index
   */
  private Map<ReactorInstance, Integer> buildTrafficClassMap() {
    Map<ReactorInstance, Integer> tcMap = new LinkedHashMap<>();
    int index = 0;
    for (ReactorInstance child : main.children) {
      if (child.getDepth() == 1) {
        tcMap.put(child, index++);
      }
    }
    return tcMap;
  }

  /**
   * Generate the bash script string for TAPRIO qdisc configuration.
   *
   * @param tcMap traffic class map (reactor -> TC index)
   * @param sendMakespans map from sending federate to its worst-case send completion time (ns)
   * @param hyperperiodNs the hyperperiod in nanoseconds
   * @return the bash script as a string
   */
  private String generateScript(
      Map<ReactorInstance, Integer> tcMap,
      Map<ReactorInstance, Long> sendMakespans,
      long hyperperiodNs) {

    int numTc = tcMap.size();

    // Sort sending federates by their worst-case send completion time.
    List<Map.Entry<ReactorInstance, Long>> sortedSenders =
        sendMakespans.entrySet().stream()
            .sorted(Comparator.comparingLong(Map.Entry::getValue))
            .collect(Collectors.toList());

    // Build gate schedule entries as consecutive non-overlapping windows.
    List<String> schedEntries = new ArrayList<>();
    long windowStart = 0;
    for (Map.Entry<ReactorInstance, Long> entry : sortedSenders) {
      ReactorInstance federate = entry.getKey();
      long windowEnd = entry.getValue();
      Integer tc = tcMap.get(federate);
      if (tc == null) continue;

      long duration = windowEnd - windowStart;
      if (duration <= 0) continue;

      // Gate mask: open only the TC for this federate.
      int gateMask = 1 << tc;
      schedEntries.add(
          String.format("    sched-entry S %02x %d \\", gateMask, duration));
      windowStart = windowEnd;
    }

    // Remaining time: all gates open for best-effort traffic.
    long remaining = hyperperiodNs - windowStart;
    if (remaining > 0) {
      int allGatesMask = (1 << numTc) - 1;
      schedEntries.add(
          String.format("    sched-entry S %02x %d \\", allGatesMask, remaining));
    }

    // Build priority-to-TC map string (identity mapping: priority i -> TC i).
    StringBuilder mapStr = new StringBuilder();
    for (int i = 0; i < 16; i++) {
      if (i > 0) mapStr.append(" ");
      mapStr.append(i < numTc ? i : 0);
    }

    // Build queue assignments string (one queue per TC).
    StringBuilder queuesStr = new StringBuilder();
    for (int i = 0; i < numTc; i++) {
      if (i > 0) queuesStr.append(" ");
      queuesStr.append("1@" + i);
    }

    // Build the script.
    StringBuilder sb = new StringBuilder();
    sb.append("#!/bin/bash\n");
    sb.append("# Auto-generated TAPRIO qdisc configuration\n");
    sb.append("# Generated from Lingua Franca static schedule DAG\n");
    sb.append("#\n");

    // Add federate-to-TC mapping as comments.
    sb.append("# Traffic class assignments:\n");
    for (Map.Entry<ReactorInstance, Integer> entry : tcMap.entrySet()) {
      sb.append(
          String.format(
              "#   TC %d: %s\n", entry.getValue(), entry.getKey().getName()));
    }
    sb.append("#\n");

    // Add send makespan info as comments.
    sb.append("# Send completion times (worst-case makespans):\n");
    for (Map.Entry<ReactorInstance, Long> entry :
        sendMakespans.entrySet().stream()
            .sorted(Comparator.comparingLong(Map.Entry::getValue))
            .collect(Collectors.toList())) {
      sb.append(
          String.format(
              "#   %s: %d ns\n", entry.getKey().getName(), entry.getValue()));
    }
    sb.append("#\n");
    sb.append(String.format("# Hyperperiod: %d ns\n", hyperperiodNs));
    sb.append("\n");

    sb.append("IFACE=${1:-eth0}\n");
    sb.append("\n");

    sb.append("tc qdisc replace dev $IFACE parent root handle 100 taprio \\\n");
    sb.append(String.format("    num_tc %d \\\n", numTc));
    sb.append(String.format("    map %s \\\n", mapStr));
    sb.append(String.format("    queues %s \\\n", queuesStr));
    sb.append("    base-time 0 \\\n");

    for (String entry : schedEntries) {
      sb.append(entry).append("\n");
    }

    sb.append("    flags 0x2\n");

    return sb.toString();
  }
}
