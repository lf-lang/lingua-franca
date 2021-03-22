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

        List<Port> networkOutputControlTriggersList = new ArrayList<Port>(
                federate.networkOutputControlTriggers);

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

            // Initialize triggers for network output control reactions
            // Initialize the triggerForNetworkOutputControlReactions for the
            // output, if any exists
            if (federate.networkOutputControlTriggers != null) {
                // Find the trigger
                if (federate.networkOutputControlTriggers.contains(input)) {
                        builder.append(
                                "_fed.triggers_for_network_output_control_reactions["
                                        + networkOutputControlTriggersList
                                        .indexOf(input)
                                        + "]= &" + nameOfSelfStruct + "->___"
                                        + input.getName() + ";\n");
                }
            }

        }

        
        return builder;
    }
}
