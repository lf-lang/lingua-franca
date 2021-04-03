package org.icyphy.federated;

import org.icyphy.generator.CGenerator;
import org.icyphy.generator.FederateInstance;
import org.icyphy.generator.ReactorInstance;
import org.icyphy.linguaFranca.Input;
import org.icyphy.linguaFranca.Port;
import org.icyphy.linguaFranca.Reactor;
import org.icyphy.linguaFranca.ReactorDecl;
import org.icyphy.linguaFranca.VarRef;
import org.icyphy.linguaFranca.Variable;
import org.icyphy.linguaFranca.TriggerRef;

public class CGeneratorExtension {

    public static StringBuilder initializeTriggerForControlReactions(
            ReactorInstance instance, FederateInstance federate, CGenerator generator) {
        StringBuilder builder = new StringBuilder();

        ReactorDecl reactorClass = instance.definition.getReactorClass();
        Reactor reactor = org.icyphy.ASTUtils.toDefinition(reactorClass);
        String nameOfSelfStruct = org.icyphy.generator.CGenerator
                .selfStructName(instance);

        // Initialize triggers for network input control reactions
        for (Port trigger : federate.networkInputControlReactionsTriggers) {
                // Check if the trigger belongs to this reactor instance                
                if (org.icyphy.ASTUtils.allReactions(reactor).stream().anyMatch(r -> {
                    return r.getTriggers().stream().anyMatch(t -> {
                        if (t instanceof VarRef) {
                            return ((VarRef)t).getVariable().equals(trigger);
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
                            + federate.networkInputControlReactionsTriggers.indexOf(trigger)
                            + "]= &"
                            + nameOfSelfStruct + "" + "->___" + trigger.getName()
                            + ";\n");
                }
        }
        
        // Initialize triggers for network input control reactions
        int index = 0;
        for (Port input : federate.networkInputPorts) {
            // Check if the input belongs to this reactor instance
            if (org.icyphy.ASTUtils.toDefinition(reactorClass).getInputs()
                    .contains((Input) input)) {
                // Initialize the network_input_port_trigger for the input, if
                // any exists
                builder.append("// Add trigger " + nameOfSelfStruct + "->___"
                        + input.getName()
                        + " to the global list of network input ports.\n"
                        + "_fed.network_input_port_triggers[" + index + "]= &"
                        + nameOfSelfStruct + "" + "->___" + input.getName()
                        + ";\n");
                index++;
            }
        }

        // The network output control reactions are always in the main federated reactor
        if (instance == generator.main) {
            nameOfSelfStruct = org.icyphy.generator.CGenerator
                    .selfStructName(instance);

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

    public static String allocateTriggersForFederate(FederateInstance instance, CGenerator generator) {
        
        StringBuilder builder = new StringBuilder();
        
        // Create the table to initialize intended tag fields to 0 between time steps.
        if (generator.isFederatedAndDecentralized() && generator.startTimeStepIsPresentCount > 0) {
            // Allocate the initial (before mutations) array of pointers to intended_tag fields.
            // There is a 1-1 map between structs containing is_present and intended_tag fields,
            // thus, we reuse startTimeStepIsPresentCount as the counter.
            builder.append("// Create the array that will contain pointers to intended_tag fields to reset on each step.\n"
                + "__intended_tag_fields_size = " + generator.startTimeStepIsPresentCount + ";\n"
                + "__intended_tag_fields = (tag_t**)malloc(__intended_tag_fields_size * sizeof(tag_t*));\n"
            );
        }
        
        if (generator.isFederated) {
            if (instance.networkInputPorts.size() > 0) {
                // Proliferate the network input port array
                builder.append("// Initialize the array of pointers to network input port triggers\n"
                    + "_fed.network_input_port_triggers_size = " + instance.networkInputPorts.size() + ";\n"
                    + "_fed.network_input_port_triggers = (trigger_t**)malloc("
                    + "_fed.network_input_port_triggers_size * sizeof(trigger_t*));\n"
                );
            }
            
            if (instance.networkInputControlReactionsTriggers.size() > 0) {
                // Proliferate the network input control reaction trigger array
                builder.append("// Initialize the array of pointers to network input port triggers\n"
                    + "_fed.triggers_for_network_input_control_reactions_size = "
                    + instance.networkInputControlReactionsTriggers.size() + ";\n"
                    + "_fed.triggers_for_network_input_control_reactions = (trigger_t**)malloc("
                    + "_fed.triggers_for_network_input_control_reactions_size * sizeof(trigger_t*)"
                    + ");\n"
                );
                
            }
        }
        
        return builder.toString();
    }
}
