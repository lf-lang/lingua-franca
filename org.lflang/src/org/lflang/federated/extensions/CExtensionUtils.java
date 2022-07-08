package org.lflang.federated.extensions;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.regex.Pattern;

import org.lflang.ASTUtils;
import org.lflang.InferredType;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty.ClockSyncMode;
import org.lflang.TimeValue;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;
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
        FederateInstance federate,
        int startTimeStepIsPresentCount,
        boolean isFederated,
        boolean isFederatedAndDecentralized
    ) {

        StringBuilder builder = new StringBuilder();

        // Create the table to initialize intended tag fields to 0 between time
        // steps.
        if (isFederatedAndDecentralized &&
            startTimeStepIsPresentCount > 0) {
            // Allocate the initial (before mutations) array of pointers to
            // intended_tag fields.
            // There is a 1-1 map between structs containing is_present and
            // intended_tag fields,
            // thus, we reuse startTimeStepIsPresentCount as the counter.
            builder.append(
                "// Create the array that will contain pointers to intended_tag fields to reset on each step.\n"
                    + "_lf_intended_tag_fields_size = "
                    + startTimeStepIsPresentCount + ";\n"
                    + "_lf_intended_tag_fields = (tag_t**)malloc("
                    + "_lf_intended_tag_fields_size * sizeof(tag_t*));\n");
        }

        if (isFederated) {
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
     * @param fileConfig
     */
    public static void generateCMakeInclude(FedFileConfig fileConfig, TargetConfig targetConfig) throws IOException {
        Path cmakeIncludePath = fileConfig.getFedSrcPath()
                                          .resolve("include" + File.pathSeparator + "extension.cmake");
        Files.createDirectories(cmakeIncludePath);

        try (var srcWriter = Files.newBufferedWriter(cmakeIncludePath)) {
            srcWriter.write("""
            target_compile_definitions(${LF_MAIN_TARGET} PUBLIC FEDERATED)
            target_compile_definitions(${LF_MAIN_TARGET} PUBLIC FEDERATED_%s)
            """.formatted(targetConfig.coordination.toString().toUpperCase()));
        }
    }

    private static boolean clockSyncIsOn() {
        return targetConfig.clockSync != ClockSyncMode.OFF
            && (!federationRTIProperties.get("host").toString().equals(currentFederate.host)
            || targetConfig.clockSyncOptions.localFederatesOn);
    }

    /**
     * Initialize clock synchronization (if enabled) and its related options for a given federate.
     *
     * Clock synchronization can be enabled using the clock-sync target property.
     * @see <a href="https://github.com/icyphy/lingua-franca/wiki/Distributed-Execution#clock-synchronization">Documentation</a>
     */
    public static void initializeClockSynchronization() {
        // Check if clock synchronization should be enabled for this federate in the first place
        if (clockSyncIsOn()) {
            System.out.println("Initial clock synchronization is enabled for federate "
                                   + currentFederate.id
            );
            if (targetConfig.clockSync == ClockSyncMode.ON) {
                if (targetConfig.clockSyncOptions.collectStats) {
                    System.out.println("Will collect clock sync statistics for federate " + currentFederate.id);
                    // Add libm to the compiler flags
                    // FIXME: This is a linker flag not compile flag but we don't have a way to add linker flags
                    // FIXME: This is probably going to fail on MacOS (especially using clang)
                    // because libm functions are builtin
                    targetConfig.compilerFlags.add("-lm");
                }
                System.out.println("Runtime clock synchronization is enabled for federate "
                                       + currentFederate.id
                );
            }
        }
    }
}