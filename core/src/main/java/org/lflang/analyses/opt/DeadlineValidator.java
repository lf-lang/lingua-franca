package org.lflang.analyses.opt;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import org.lflang.MessageReporter;
import org.lflang.TimeValue;
import org.lflang.analyses.dag.Dag;
import org.lflang.analyses.dag.DagNode;
import org.lflang.analyses.pretvm.PretVmObjectFile;
import org.lflang.analyses.pretvm.instructions.Instruction;
import org.lflang.analyses.pretvm.profiles.FlexPRETProfile;
import org.lflang.analyses.statespace.StateSpaceExplorer.Phase;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.PlatformProperty;
import org.lflang.target.property.PlatformProperty.PlatformOptions;
import org.lflang.target.property.SchedulerProperty;
import org.lflang.target.property.type.PlatformType.Platform;

public class DeadlineValidator {
    /**
     * Validate the deadlines in the DAG of the given PretVM object file.
     * @param messageReporter The message reporter to report any errors/warnings to.
     * @param targetConfig The target configuration.
     * @param objectFile The PretVM object file to validate.
     * @return True if the deadlines are met, false otherwise.
     */
    public static boolean validateDeadline(MessageReporter messageReporter, TargetConfig targetConfig, PretVmObjectFile objectFile) {
        // Get the phase from the object file.
        Phase phase = objectFile.getFragment().getPhase();
        // Get the DAG from the object file
        Dag dag = objectFile.getDag();
        // Map a node to a makespan up to that node
        Map<DagNode, TimeValue> makespan = new HashMap<DagNode, TimeValue>();
        // Flag to indicate if the deadline is met
        boolean deadlineMet = true;
        // Perform a topological sort of the DAG and calculate the makespan.
        for (DagNode node : dag.getTopologicalSort()) {
            // The head node must be a SYNC node, so the makespan is 0.
            if (node == dag.head) {
                makespan.put(node, TimeValue.ZERO);
                continue;
            }
            // Look up the makespan of the predecessors of the node.
            List<DagNode> upstreamNodes = dag.getUpstreamNodes(node);
            TimeValue maxUpstreamMakespan = upstreamNodes.stream().map(it -> makespan.get(it)).max(TimeValue::compareTo).get();
            
            // Update the makespan map based on the node type.
            switch (node.nodeType) {
                case DUMMY:
                    // FIXME: The DUMMY node's WCET is stored in the
                    // timeStep field. This is very ugly and need to be
                    // refactored.
                    makespan.put(node, maxUpstreamMakespan.add(node.timeStep));
                    break;
                case SYNC:
                    // A SYNC node has a WCET of 0, so just add the max
                    // upstream makespan.
                    makespan.put(node, maxUpstreamMakespan);
                    break;
                case REACTION:
                    // If the node is a reaction, add the reaction WCET
                    // to the makespan.
                    // Currently, we only support a single WCET per node.
                    if (node.nodeReaction.wcets.size() > 1) {
                        messageReporter.nowhere().warning("DeadlineValidator: Node " + node + " has more than one WCET.");
                    }
                    TimeValue makespanUntilNode = maxUpstreamMakespan.add(node.nodeReaction.wcets.get(0));
                    // For each PretVM instructions generated by the
                    // node, add up their overhead.
                    // Find all instructions own by the node's worker.
                    List<Instruction> workInsts = objectFile.getContent().get(node.getWorker());
                    // Find the instructions that belong to this node only.
                    List<Instruction> nodeInsts = node.filterInstructions(workInsts);
                    // Add up the overhead of the instructions.
                    Platform platform = targetConfig.getOrDefault(PlatformProperty.INSTANCE).platform();
                    // Add instruction overhead based on the platform used.
                    if (platform != null) {
                        switch (platform) {
                            case AUTO: break;
                            case FLEXPRET:
                                for (Instruction inst : nodeInsts) {
                                    makespanUntilNode = makespanUntilNode.add(FlexPRETProfile.getInstWCET(inst.getOpcode()));
                                }
                                break;
                            default:
                                messageReporter.nowhere().error("DeadlineValidator: Unknown platform " + platform);
                        }
                    }
                    makespan.put(node, makespanUntilNode);
                    break;
                default:
                    messageReporter.nowhere().error("DeadlineValidator: Unknown node type " + node.nodeType);
                    return false;
            }

            // If a SYNC node is encountered, check if the makespan
            // exceeds the deadline.
            // At the moment, TimeValue has a "saturation" semantics,
            // meaning that if the sum of two time values exceeds the
            // maximum time value, the sum becomes the maximum time
            // value. This semantics helps with deadline checking here,
            // If any reaction has an unknown WCET
            // (represented as TimeValue.MAX_VALUE), this pulls up the
            // makespan along the DAG node chain to TimeValue.MAX_VALUE. 
            // For real deadlines, i.e., SYNC nodes with timestamp <
            // TimeValue.MAX_VALUE, deadline violations can be easily
            // detected using the compareTo() method. For fake
            // deadlines, SYNC nodes with timestamp ==
            // TimeValue.MAX_VALUE (these SYNC nodes are simply there to
            // represent the end of a hyperperiod / phase), 
            // the saturation semantics make sure that compareTo()
            // returns 0 and no deadline violations are returned.
            if (node.nodeType == DagNode.dagNodeType.SYNC) {
                if (makespan.get(node).compareTo(node.timeStep) > 0) {
                    messageReporter.nowhere().warning("DeadlineValidator: Deadline violation detected in phase " + phase + " at node " + node);
                    deadlineMet = false;
                }
            }

            // FIXME: Generate a DOT file that shows the makespan.
        }

        return deadlineMet;
    }
}
