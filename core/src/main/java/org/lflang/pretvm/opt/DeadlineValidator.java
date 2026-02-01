package org.lflang.pretvm.opt;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.lflang.MessageReporter;
import org.lflang.TimeValue;
import org.lflang.pretvm.ExecutionPhase;
import org.lflang.pretvm.PartialSchedule;
import org.lflang.pretvm.dag.Dag;
import org.lflang.pretvm.dag.DagNode;
import org.lflang.pretvm.dag.IntervalNode;
import org.lflang.pretvm.dag.JobNode;
import org.lflang.pretvm.dag.TimeNode;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.PlatformProperty;
import org.lflang.target.property.type.PlatformType.Platform;

/** Validates that reaction execution meets deadlines imposed by SYNC nodes in the DAG. */
public class DeadlineValidator {

  /**
   * Validate the deadlines in the DAG of the given partial schedule.
   *
   * @return true if all deadlines are met, false otherwise.
   */
  public static boolean validateDeadline(
      MessageReporter messageReporter, TargetConfig targetConfig, PartialSchedule schedule) {
    ExecutionPhase phase = schedule.getPhase();
    Dag dag = schedule.getDag();
    Map<DagNode, TimeValue> makespan = new HashMap<>();
    boolean deadlineMet = true;

    for (DagNode node : dag.getTopologicalSort()) {
      if (node == dag.start) {
        makespan.put(node, TimeValue.ZERO);
        continue;
      }

      List<DagNode> upstreamNodes = dag.getUpstreamNodes(node);
      TimeValue maxUpstreamMakespan =
          upstreamNodes.stream().map(makespan::get).max(TimeValue::compareTo).get();

      if (node instanceof IntervalNode intervalNode) {
        // FIXME: The IntervalNode's WCET is stored in the interval field. Needs refactoring.
        makespan.put(node, maxUpstreamMakespan.add(intervalNode.getInterval()));
      } else if (node instanceof TimeNode) {
        // A TimeNode (SYNC) has a WCET of 0.
        makespan.put(node, maxUpstreamMakespan);
      } else if (node instanceof JobNode jobNode) {
        TimeValue makespanUntilNode = maxUpstreamMakespan.add(jobNode.getReaction().wcet);

        // Add instruction overhead based on the platform.
        Platform platform = targetConfig.getOrDefault(PlatformProperty.INSTANCE).platform();
        if (platform != null) {
          switch (platform) {
            case AUTO:
              break;
            case FLEXPRET:
              // FIXME: Add platform-specific instruction overhead once
              // a FlexPRET profile is available in this package.
              break;
            default:
              messageReporter.nowhere().error("DeadlineValidator: Unknown platform " + platform);
          }
        }
        makespan.put(node, makespanUntilNode);
      } else {
        messageReporter
            .nowhere()
            .error("DeadlineValidator: Unknown node type " + node.getClass().getSimpleName());
        return false;
      }

      // Check deadline at SYNC (TimeNode) boundaries.
      if (node instanceof TimeNode timeNode) {
        if (makespan.get(node).compareTo(timeNode.getTime()) > 0) {
          messageReporter
              .nowhere()
              .warning(
                  "DeadlineValidator: Deadline violation detected in phase "
                      + phase
                      + " at node "
                      + node);
          deadlineMet = false;
        }
      }
    }

    return deadlineMet;
  }
}
