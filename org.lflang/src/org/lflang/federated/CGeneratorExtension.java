package org.lflang.federated;

import java.util.LinkedHashSet;

import org.lflang.ASTUtils;
import org.lflang.generator.CGenerator;
import org.lflang.generator.FederateInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.meta.Input;
import org.lflang.meta.Port;
import org.lflang.meta.Reactor;
import org.lflang.meta.ReactorDecl;
import org.lflang.meta.VarRef;

public class CGeneratorExtension {

    /**
     * Generate C code that allocates sufficient memory for the following three critical 
     * data structures that support network control reactions:
     *  - triggers_for_network_input_control_reactions: These are triggers that are used
     *     at runtime to insert network input control reactions into the reaction queue.
     *     There could be multiple network input control reactions for one network input
     *     at multiple levels in the hierarchy.
     *  - network_input_port_triggers: These triggers are exclusively used to communicate
     *     the status of network ports between the receiver logic (in federate.c) and the
     *     network input control reactions at any level in the hierarchy.
     *  - trigger_for_network_output_control_reactions: Triggers for network output control
     *     reactions, which are unique per each output port. There could be multiple network
     *     output control reactions for each network output port if it is connected to multiple
     *     downstream federates.
     * @param federate The top-level federate instance
     * @param generator The instance of the CGenerator passed to keep this extension function
     *  static.
     * @return A string that allocates memory for the aforementioned three structures.
     */
    public static String allocateTriggersForFederate(FederateInstance federate,
            CGenerator generator) {

        StringBuilder builder = new StringBuilder();

        // Create the table to initialize intended tag fields to 0 between time
        // steps.
        if (generator.isFederatedAndDecentralized()
                && generator.startTimeStepIsPresentCount > 0) {
            // Allocate the initial (before mutations) array of pointers to
            // intended_tag fields.
            // There is a 1-1 map between structs containing is_present and
            // intended_tag fields,
            // thus, we reuse startTimeStepIsPresentCount as the counter.
            builder.append(
                    "// Create the array that will contain pointers to intended_tag fields to reset on each step.\n"
                            + "__intended_tag_fields_size = "
                            + generator.startTimeStepIsPresentCount + ";\n"
                            + "__intended_tag_fields = (tag_t**)malloc(__intended_tag_fields_size * sizeof(tag_t*));\n");
        }

        if (generator.isFederated) {
            if (federate.networkInputPorts.size() > 0) {
                // Proliferate the network input port array
                builder.append(
                        "// Initialize the array of pointers to network input port triggers\n"
                                + "_fed.network_input_port_triggers_size = "
                                + federate.networkInputPorts.size() + ";\n"
                                + "_fed.network_input_port_triggers = (trigger_t**)malloc("
                                + "_fed.network_input_port_triggers_size * sizeof(trigger_t*));\n");
            }

            if (federate.networkInputControlReactionsTriggers.size() > 0) {
                // Proliferate the network input control reaction trigger array
                builder.append(
                        "// Initialize the array of pointers to network input port triggers\n"
                                + "_fed.triggers_for_network_input_control_reactions_size = "
                                + federate.networkInputControlReactionsTriggers
                                .size()
                                + ";\n"
                                + "_fed.triggers_for_network_input_control_reactions = (trigger_t**)malloc("
                                + "_fed.triggers_for_network_input_control_reactions_size * sizeof(trigger_t*)"
                                + ");\n");

            }
        }

        return builder.toString();
    }
    
    /**
     * Generate C code that initializes three critical structures that support network 
     * control reactions:
     *  - triggers_for_network_input_control_reactions: These are triggers that are used
     *     at runtime to insert network input control reactions into the reaction queue.
     *     There could be multiple network input control reactions for one network input
     *     at multiple levels in the hierarchy.
     *  - network_input_port_triggers: These triggers are exclusively used to communicate
     *     the status of network ports between the receiver logic (in federate.c) and the
     *     network input control reactions at any level in the hierarchy.
     *  - trigger_for_network_output_control_reactions: Triggers for network output control
     *     reactions, which are unique per each output port. There could be multiple network
     *     output control reactions for each network output port if it is connected to multiple
     *     downstream federates.
     * @param instance The reactor instance that is at any level of the hierarchy within the
     *  federate.
     * @param federate The top-level federate
     * @param generator The instance of the CGenerator passed to keep this extension function
     *  static.
     * @return A string that initializes the aforementioned three structures.
     */
    public static StringBuilder initializeTriggerForControlReactions(
            ReactorInstance instance, FederateInstance federate,
            CGenerator generator) {
        StringBuilder builder = new StringBuilder();

        ReactorDecl reactorClass = instance.definition.getReactorClass();
        Reactor reactor = ASTUtils.toDefinition(reactorClass);
        String nameOfSelfStruct = CGenerator.selfStructName(instance);

        // Initialize triggers for network input control reactions
        for (Port trigger : federate.networkInputControlReactionsTriggers) {
            // Check if the trigger belongs to this reactor instance
            if (ASTUtils.allReactions(reactor).stream()
                    .anyMatch(r -> {
                        return r.getTriggers().stream().anyMatch(t -> {
                            if (t instanceof VarRef) {
                                return ((VarRef) t).getVariable()
                                        .equals(trigger);
                            } else {
                                return false;
                            }
                        });
                    })) {
                // Initialize the network_input_port_trigger for the input, if
                // any exists
                builder.append("// Add trigger " + nameOfSelfStruct + "->___"
                        + trigger.getName()
                        + " to the global list of network input ports.\n"
                        + "_fed.triggers_for_network_input_control_reactions["
                        + federate.networkInputControlReactionsTriggers
                        .indexOf(trigger)
                        + "]= &" + nameOfSelfStruct + "" + "->___"
                        + trigger.getName() + ";\n");
            }
        }

        // Initialize triggers for network input control reactions
        int index = 0;
        LinkedHashSet<Port> alreadyProcessedPorts = new LinkedHashSet<Port>();
        for (Port input : federate.networkInputPorts) {
            // networkInputPorts is a linked list that can contain
            // duplicate ports in case input is a multiport. We
            // would only need to generate code for a multiport once,
            // but we need to keep track of the index because network
            // multiports are indexed according to the index of the multiport.
            // For example, an input multiport a[4] will translate to the following
            // network port indexes: 0, 1, 2, 3
            if (!alreadyProcessedPorts.contains(input)) {
                alreadyProcessedPorts.add(input);
                // Check if the input belongs to this reactor instance
                if (ASTUtils.toDefinition(reactorClass).getInputs()
                        .contains((Input) input)) {
                    if (generator.isMultiport(input)) {
                        // Initialize the network_input_port_trigger for the
                        // input, if any exists
                        builder.append(nameOfSelfStruct + "" + "->___"
                                + input.getName() + "_network_port_status \n"
                                + " = malloc(" + nameOfSelfStruct + "->__"
                                + input.getName()
                                + "__width * sizeof(trigger_t));\n");

                        builder.append("// Add trigger " + nameOfSelfStruct
                                + "->___" + input.getName()
                                + " to the global list of network input ports.\n"
                                + "for (int i=0; i < " + nameOfSelfStruct
                                + "->__" + input.getName() + "__width; i++) {\n"
                                + "\t_fed.network_input_port_triggers[" + index
                                + "+i]= &" + nameOfSelfStruct + "" + "->___"
                                + input.getName() + "_network_port_status[i];\n"
                                + "}\n");
                    } else {
                        // Initialize the network_input_port_trigger for the
                        // input, if
                        // any exists
                        builder.append("// Add trigger " + nameOfSelfStruct
                                + "->___" + input.getName()
                                + " to the global list of network input ports.\n"
                                + "_fed.network_input_port_triggers[" + index
                                + "]= &" + nameOfSelfStruct + "" + "->___"
                                + input.getName() + ";\n");
                    }
                }
            }
            index++;
        }

        // The network output control reactions are always in the main federated
        // reactor
        if (instance == generator.main) {
            nameOfSelfStruct = CGenerator.selfStructName(instance);

            // Initialize triggers for network output control reactions
            // Initialize the triggerForNetworkOutputControlReactions for the
            // output, if any exists
            if (federate.networkOutputControlReactionsTrigger != null) {
                builder.append(
                        "_fed.trigger_for_network_output_control_reactions=&"
                                + nameOfSelfStruct
                                + "->___outputControlReactionTrigger;\n");
            }
        }

        return builder;
    }

    /**
     * Create a port status field variable for a network input port "input" in
     * the self struct of a reactor.
     * 
     * @param input     The network input port
     * @param generator The instance of the CGenerator
     * @return A string containing the appropriate variable
     */
    public static String createPortStatusFieldForInput(Input input,
            CGenerator generator) {
        StringBuilder builder = new StringBuilder();
        // Check if the port is a multiport
        if (generator.isMultiport(input)) {
            // If it is a multiport, then create an auxiliary list of port
            // triggers for each channel of
            // the multiport to keep track of the status of each channel
            // individually
            builder.append("trigger_t* ___" + input.getName()
            + "_network_port_status;\n");
        } else {
            // If it is not a multiport, then we could re-use the port trigger,
            // and nothing needs to be
            // done
        }
        return builder.toString();
    }
}
