package org.icyphy.federated;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.icyphy.generator.FederateInstance;
import org.icyphy.generator.ReactorInstance;
import org.icyphy.linguaFranca.Input;
import org.icyphy.linguaFranca.Output;
import org.icyphy.linguaFranca.Port;
import org.icyphy.linguaFranca.Reaction;
import org.icyphy.linguaFranca.ReactorDecl;
import org.icyphy.linguaFranca.VarRef;

public class CGeneratorExtension {

    public static StringBuilder initializeTriggerForControlReactions(
            ReactorInstance instance, FederateInstance federate) {
        StringBuilder builder = new StringBuilder();

        List<Port> networkInputPorts = new ArrayList<Port>(
                federate.networkInputPorts);

        ReactorDecl reactorClass = instance.definition.getReactorClass();
        String nameOfSelfStruct = org.icyphy.generator.CGenerator
                .selfStructName(instance);

        // Initialize triggers for network input control reactions
        for (Input input : org.icyphy.ASTUtils.toDefinition(reactorClass)
                .getInputs()) {
            // Initialize the network_input_port_trigger for the input, if any
            // exists
            if (federate.networkInputPorts != null) {
                if (federate.networkInputPorts.contains(input)) {
                    builder.append("// Add trigger " + nameOfSelfStruct
                            + "->___" + input.getName()
                            + " to the global list of network input ports.\n"
                            + "_fed.network_input_port_triggers["
                            + networkInputPorts.indexOf(input) + "]= &"
                            + nameOfSelfStruct + "" + "->___" + input.getName()
                            + ";\n");
                }
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
