package org.lflang.federated.extensions;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.regex.Pattern;

import org.lflang.ASTUtils;
import org.lflang.InferredType;
import org.lflang.TargetConfig.ClockSyncOptions;
import org.lflang.TargetProperty.ClockSyncMode;
import org.lflang.TimeValue;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.c.CTypes;
import org.lflang.generator.c.CUtil;
import org.lflang.lf.Action;
import org.lflang.lf.Expression;
import org.lflang.lf.Input;
import org.lflang.lf.ParameterReference;
import org.lflang.lf.Reactor;
import org.lflang.lf.ReactorDecl;
import org.lflang.lf.VarRef;

public class CExtensionUtils {

    // Regular expression pattern for shared_ptr types.
    static final Pattern sharedPointerVariable = Pattern.compile("^(/\\*.*?\\*/)?std::shared_ptr<(?<type>((/\\*.*?\\*/)?(\\S+))+)>$");

    /**
     * Generate C code that allocates sufficient memory for the following two
     * critical data structures that support network control reactions:
     * - triggers_for_network_input_control_reactions: These are triggers that
     * are
     * used at runtime to insert network input control reactions into the
     * reaction queue.
     * - trigger_for_network_output_control_reactions: Triggers for
     * network output control reactions, which are unique per each output port.
     * There could be multiple network output control reactions for each
     * network
     * output port if it is connected to multiple downstream federates.
     *
     * @param federate The top-level federate instance
     * @return A string that allocates memory for the aforementioned three
     * structures.
     */
    public static String allocateTriggersForFederate(
        FederateInstance federate
    ) {

        StringBuilder builder = new StringBuilder();
        if (federate.networkInputControlReactionsTriggers.size() > 0) {
            // Proliferate the network input control reaction trigger array
            builder.append(
                "// Initialize the array of pointers to network input port triggers\n"
                    + "_fed.triggers_for_network_input_control_reactions_size = "
                    + federate.networkInputControlReactionsTriggers.size()
                    + ";\n"
                    + "_fed.triggers_for_network_input_control_reactions = (trigger_t**)malloc("
                    + "_fed.triggers_for_network_input_control_reactions_size * sizeof(trigger_t*)"
                    + ");\n");

        }
        return builder.toString();
    }

    /**
     * Generate C code that initializes three critical structures that support
     * network control reactions:
     *  - triggers_for_network_input_control_reactions: These are triggers that are
     *  used at runtime to insert network input control reactions into the
     *  reaction queue. There could be multiple network input control reactions
     *  for one network input at multiple levels in the hierarchy.
     *  - trigger_for_network_output_control_reactions: Triggers for
     *  network output control reactions, which are unique per each output port.
     *  There could be multiple network output control reactions for each network
     *  output port if it is connected to multiple downstream federates.
     *
     * @param instance  The reactor instance that is at any level of the
     *                  hierarchy within the federate.
     * @param federate  The top-level federate
     * @return A string that initializes the aforementioned three structures.
     */
    public static String initializeTriggerForControlReactions(
            ReactorInstance instance,
            ReactorInstance main,
            FederateInstance federate
    ) {
        StringBuilder builder = new StringBuilder();
        // The network control reactions are always in the main federated
        // reactor
        if (instance != main) {
            return "";
        }

        ReactorDecl reactorClass = instance.getDefinition().getReactorClass();
        Reactor reactor = ASTUtils.toDefinition(reactorClass);
        String nameOfSelfStruct = CUtil.reactorRef(instance);

        // Initialize triggers for network input control reactions
        for (Action trigger : federate.networkInputControlReactionsTriggers) {
            // Check if the trigger belongs to this reactor instance
            if (ASTUtils.allReactions(reactor).stream().anyMatch(r -> {
                return r.getTriggers().stream().anyMatch(t -> {
                    if (t instanceof VarRef) {
                        return ((VarRef) t).getVariable().equals(trigger);
                    } else {
                        return false;
                    }
                });
            })) {
                // Initialize the triggers_for_network_input_control_reactions for the input
                builder.append("// Add trigger " + nameOfSelfStruct + "->_lf__"
                        + trigger.getName()
                        + " to the global list of network input ports.\n"
                        + "_fed.triggers_for_network_input_control_reactions["
                        + federate.networkInputControlReactionsTriggers.indexOf(trigger)
                        + "]= &" + nameOfSelfStruct + "" + "->_lf__"
                        + trigger.getName() + ";\n");
            }
        }

        nameOfSelfStruct = CUtil.reactorRef(instance);

        // Initialize the trigger for network output control reactions if it doesn't exist.
        if (federate.networkOutputControlReactionsTrigger != null) {
            builder.append("_fed.trigger_for_network_output_control_reactions=&"
                    + nameOfSelfStruct
                    + "->_lf__outputControlReactionTrigger;\n");
        }

        return builder.toString();
    }

    /**
     * Create a port status field variable for a network input port "input" in
     * the self struct of a reactor.
     *
     * @param input     The network input port
     * @return A string containing the appropriate variable
     */
    public static String createPortStatusFieldForInput(Input input) {
        StringBuilder builder = new StringBuilder();
        // Check if the port is a multiport
        if (ASTUtils.isMultiport(input)) {
            // If it is a multiport, then create an auxiliary list of port
            // triggers for each channel of
            // the multiport to keep track of the status of each channel
            // individually
            builder.append("trigger_t* _lf__" + input.getName()
            + "_network_port_status;\n");
        } else {
            // If it is not a multiport, then we could re-use the port trigger,
            // and nothing needs to be
            // done
        }
        return builder.toString();
    }

    /**
     * Given a connection 'delay' predicate, return a string that represents the
     * interval_t value of the additional delay that needs to be applied to the
     * outgoing message.
     *
     * The returned additional delay in absence of after on network connection
     * (i.e., if delay is passed as a null) is  NEVER. This has a special
     * meaning in C library functions that send network messages that carry
     * timestamps (@see send_timed_message and send_port_absent_to_federate
     * in lib/core/federate.c). In this case, the sender will send its current
     * tag as the timestamp of the outgoing message without adding a microstep delay.
     * If the user has assigned an after delay to the network connection (that
     * can be zero) either as a time value (e.g., 200 msec) or as a literal
     * (e.g., a parameter), that delay in nsec will be returned.
     *
     * @param delay
     * @return
     */
    public static String getNetworkDelayLiteral(Expression delay) {
        String additionalDelayString = "NEVER";
        if (delay != null) {
            TimeValue tv;
            if (delay instanceof ParameterReference) {
                // The parameter has to be parameter of the main reactor.
                // And that value has to be a Time.
                tv = ASTUtils.getDefaultAsTimeValue(((ParameterReference)delay).getParameter());
            } else {
                tv = ASTUtils.getLiteralTimeValue(delay);
            }
            additionalDelayString = Long.toString(tv.toNanoSeconds());
        }
        return additionalDelayString;
    }

    static boolean isSharedPtrType(InferredType type, CTypes types) {
        return !type.isUndefined() && sharedPointerVariable.matcher(types.getTargetType(type)).find();
    }

    /**
     * Generate a file to be included by CMake
     *
     * @param fileConfig
     * @param federate
     */
    public static void generateCMakeInclude(FedFileConfig fileConfig, FederateInstance federate) throws IOException {
        Files.createDirectories(fileConfig.getFedSrcPath().resolve("include"));

        Path cmakeIncludePath = fileConfig.getFedSrcPath()
                                          .resolve("include" + File.separator + federate.name + "_extension.cmake");

        var advanceMessageInterval = federate.targetConfig.coordinationOptions.advance_message_interval;

        try (var srcWriter = Files.newBufferedWriter(cmakeIncludePath)) {
            // FIXME: translate compileDefinitions to the cmake-include string
            //// Add compile definitions for federated execution
            //targetConfig.compileDefinitions.put("FEDERATED", "");
            //if (targetConfig.coordination == CoordinationType.CENTRALIZED) {
            //    // The coordination is centralized.
            //    targetConfig.compileDefinitions.put("FEDERATED_CENTRALIZED", "");
            //} else if (targetConfig.coordination == CoordinationType.DECENTRALIZED) {
            //    // The coordination is decentralized
            //    targetConfig.compileDefinitions.put("FEDERATED_DECENTRALIZED", "");
            //}

            srcWriter.write("""
                target_compile_definitions(${LF_MAIN_TARGET} PUBLIC FEDERATED)
                target_compile_definitions(${LF_MAIN_TARGET} PUBLIC FEDERATED_%s)
                            
                # Convey to the C runtime the required number of worker threads to
                # handle network input control reactions.
                target_compile_definitions(${LF_MAIN_TARGET} PUBLIC WORKERS_NEEDED_FOR_FEDERATE %s)
                """.formatted(
                federate.targetConfig.coordination.toString().toUpperCase(),
                Integer.toString(federate.networkMessageActions.size()))
            );
        }

        federate.targetConfig.cmakeIncludes.add(cmakeIncludePath.toString());
    }

    static boolean clockSyncIsOn(FederateInstance federate, LinkedHashMap<String, Object> federationRTIProperties) {
        return federate.targetConfig.clockSync != ClockSyncMode.OFF
            && (!federationRTIProperties.get("host").toString().equals(federate.host)
            || federate.targetConfig.clockSyncOptions.localFederatesOn);
    }

    /**
     * Initialize clock synchronization (if enabled) and its related options for a given federate.
     *
     * Clock synchronization can be enabled using the clock-sync target property.
     * @see <a href="https://github.com/icyphy/lingua-franca/wiki/Distributed-Execution#clock-synchronization">Documentation</a>
     */
    public static void initializeClockSynchronization(FederateInstance federate, LinkedHashMap<String, Object> federationRTIProperties) {
        // Check if clock synchronization should be enabled for this federate in the first place
        if (clockSyncIsOn(federate, federationRTIProperties)) {
            System.out.println("Initial clock synchronization is enabled for federate "
                                   + federate.id
            );
            if (federate.targetConfig.clockSync == ClockSyncMode.ON) {
                if (federate.targetConfig.clockSyncOptions.collectStats) {
                    System.out.println("Will collect clock sync statistics for federate " + federate.id);
                    // Add libm to the compiler flags
                    // FIXME: This is a linker flag not compile flag but we don't have a way to add linker flags
                    // FIXME: This is probably going to fail on MacOS (especially using clang)
                    // because libm functions are builtin
                    federate.targetConfig.compilerFlags.add("-lm");
                }
                System.out.println("Runtime clock synchronization is enabled for federate "
                                       + federate.id
                );
            }
        }
    }

    /**
     * Generate code that sends the neighbor structure message to the RTI.
     * @see MSG_TYPE_NEIGHBOR_STRUCTURE in net_common.h
     *
     * @param federate The federate that is sending its neighbor structure
     */
    public static String generateFederateNeighborStructure(FederateInstance federate) {
        var code = new CodeBuilder();
        code.pr(String.join("\n",
                            "/**",
                            "* Generated function that sends information about connections between this federate and",
                            "* other federates where messages are routed through the RTI. Currently, this",
                            "* only includes logical connections when the coordination is centralized. This",
                            "* information is needed for the RTI to perform the centralized coordination.",
                            "* @see MSG_TYPE_NEIGHBOR_STRUCTURE in net_common.h",
                            "*/",
                            "void send_neighbor_structure_to_RTI(int rti_socket) {"
        ));
        code.indent();
        // Initialize the array of information about the federate's immediate upstream
        // and downstream relayed (through the RTI) logical connections, to send to the
        // RTI.
        code.pr(String.join("\n",
                            "interval_t candidate_tmp;",
                            "size_t buffer_size = 1 + 8 + ",
                            "                "+federate.dependsOn.keySet().size()+" * ( sizeof(uint16_t) + sizeof(int64_t) ) +",
                            "                "+federate.sendsTo.keySet().size()+" * sizeof(uint16_t);",
                            "unsigned char buffer_to_send[buffer_size];",
                            "",
                            "size_t message_head = 0;",
                            "buffer_to_send[message_head] = MSG_TYPE_NEIGHBOR_STRUCTURE;",
                            "message_head++;",
                            "encode_int32((int32_t)"+federate.dependsOn.keySet().size()+", &(buffer_to_send[message_head]));",
                            "message_head+=sizeof(int32_t);",
                            "encode_int32((int32_t)"+federate.sendsTo.keySet().size()+", &(buffer_to_send[message_head]));",
                            "message_head+=sizeof(int32_t);"
        ));

        if (!federate.dependsOn.keySet().isEmpty()) {
            // Next, populate these arrays.
            // Find the minimum delay in the process.
            // FIXME: Zero delay is not really the same as a microstep delay.
            for (FederateInstance upstreamFederate : federate.dependsOn.keySet()) {
                code.pr(String.join("\n",
                                    "encode_uint16((uint16_t)"+upstreamFederate.id+", &(buffer_to_send[message_head]));",
                                    "message_head += sizeof(uint16_t);"
                ));
                // The minimum delay calculation needs to be made in the C code because it
                // may depend on parameter values.
                // FIXME: These would have to be top-level parameters, which don't really
                // have any support yet. Ideally, they could be overridden on the command line.
                // When that is done, they will need to be in scope here.
                var delays = federate.dependsOn.get(upstreamFederate);
                if (delays != null) {
                    // There is at least one delay, so find the minimum.
                    // If there is no delay at all, this is encoded as NEVER.
                    code.pr("candidate_tmp = FOREVER;");
                    for (Expression delay : delays) {
                        if (delay == null) {
                            // Use NEVER to encode no delay at all.
                            code.pr("candidate_tmp = NEVER;");
                        } else {
                            var delayTime = GeneratorBase.getTargetTime(delay);
                            if (delay instanceof ParameterReference) {
                                // The delay is given as a parameter reference. Find its value.
                                final var param = ((ParameterReference)delay).getParameter();
                                delayTime = GeneratorBase.timeInTargetLanguage(ASTUtils.getDefaultAsTimeValue(param));
                            }
                            code.pr(String.join("\n",
                                                "if ("+delayTime+" < candidate_tmp) {",
                                                "    candidate_tmp = "+delayTime+";",
                                                "}"
                            ));
                        }
                    }
                    code.pr(String.join("\n",
                                        "encode_int64((int64_t)candidate_tmp, &(buffer_to_send[message_head]));",
                                        "message_head += sizeof(int64_t);"
                    ));
                } else {
                    // Use NEVER to encode no delay at all.
                    code.pr(String.join("\n",
                                        "encode_int64(NEVER, &(buffer_to_send[message_head]));",
                                        "message_head += sizeof(int64_t);"
                    ));
                }
            }
        }

        // Next, set up the downstream array.
        if (!federate.sendsTo.keySet().isEmpty()) {
            // Next, populate the array.
            // Find the minimum delay in the process.
            // FIXME: Zero delay is not really the same as a microstep delay.
            for (FederateInstance downstreamFederate : federate.sendsTo.keySet()) {
                code.pr(String.join("\n",
                                    "encode_uint16("+downstreamFederate.id+", &(buffer_to_send[message_head]));",
                                    "message_head += sizeof(uint16_t);"
                ));
            }
        }
        code.pr(String.join("\n",
                            "write_to_socket_errexit(",
                            "    rti_socket, ",
                            "    buffer_size,",
                            "    buffer_to_send,",
                            "    \"Failed to send the neighbor structure message to the RTI.\"",
                            ");"
        ));
        code.unindent();
        code.pr("}");
        return code.toString();
    }


    public static List<String> getFederatedFiles() {
        return List.of(
            "federated/net_util.c",
            "federated/net_util.h",
            "federated/net_common.h",
            "federated/federate.c",
            "federated/federate.h",
            "federated/clock-sync.h",
            "federated/clock-sync.c"
        );
    }

    /**
     * Surround {@code code} with blocks to ensure that code only executes
     * if the program is federated.
     */
    public static String surroundWithIfFederated(String code) {
        return """
            #ifdef FEDERATED
            %s
            #endif // FEDERATED
            """.formatted(code);
    }

    /**
     * Surround {@code code} with blocks to ensure that code only executes
     * if the program is federated and has a centralized coordination.
     */
    public static String surroundWithIfFederatedCentralized(String code) {
        return """
            #ifdef FEDERATED_CENTRALIZED
            %s
            #endif // FEDERATED_CENTRALIZED
            """.formatted(code);
    }

    /**
     * Surround {@code code} with blocks to ensure that code only executes
     * if the program is federated and has a decentralized coordination.
     */
    public static String surroundWithIfFederatedDecentralized(String code) {
        return """
            #ifdef FEDERATED_DECENTRALIZED
            %s
            #endif // FEDERATED_DECENTRALIZED
            """.formatted(code);
    }

    /**
     * Initialize clock synchronization (if enabled) and its related options for a given federate.
     *
     * Clock synchronization can be enabled using the clock-sync target property.
     * @see <a href="https://github.com/icyphy/lingua-franca/wiki/Distributed-Execution#clock-synchronization">Documentation</a>
     */
    public static String generateClockSyncDefineDirective(
        ClockSyncMode mode,
        ClockSyncOptions options
    ) {
        List<String> code = new ArrayList<>(List.of(
            "#define _LF_CLOCK_SYNC_INITIAL",
            "#define _LF_CLOCK_SYNC_PERIOD_NS " + GeneratorBase.timeInTargetLanguage(options.period),
            "#define _LF_CLOCK_SYNC_EXCHANGES_PER_INTERVAL " + options.trials,
            "#define _LF_CLOCK_SYNC_ATTENUATION " + options.attenuation
        ));
        if (mode == ClockSyncMode.ON) {
            code.add("#define _LF_CLOCK_SYNC_ON");
            if (options.collectStats) {
                code.add("#define _LF_CLOCK_SYNC_COLLECT_STATS");
            }
        }
        return String.join("\n", code);
    }
}