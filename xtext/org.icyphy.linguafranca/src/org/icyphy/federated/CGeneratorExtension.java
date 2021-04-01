package org.icyphy.federated;

import org.icyphy.generator.FederateInstance;
import org.icyphy.generator.ReactorInstance;
import org.icyphy.linguaFranca.Input;
import org.icyphy.linguaFranca.Port;
import org.icyphy.linguaFranca.ReactorDecl;

public class CGeneratorExtension {

    public static StringBuilder initializeTriggerForControlReactions(
            ReactorInstance instance, FederateInstance federate) {
        StringBuilder builder = new StringBuilder();

        ReactorDecl reactorClass = instance.definition.getReactorClass();
        String nameOfSelfStruct = org.icyphy.generator.CGenerator
                .selfStructName(instance);

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

        if (instance.parent != null) {
            nameOfSelfStruct = org.icyphy.generator.CGenerator
                    .selfStructName(instance.parent);

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
}
