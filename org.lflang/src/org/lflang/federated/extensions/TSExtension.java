package org.lflang.federated.extensions;

import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.stream.Collectors;

import org.lflang.ErrorReporter;
import org.lflang.InferredType;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.TimeValue;
import org.lflang.federated.generator.FedConnectionInstance;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.lf.Action;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;

public class TSExtension implements FedTargetExtension {
    @Override
    public void initializeTargetConfig(LFGeneratorContext context, int numOfFederates, FederateInstance federate, FedFileConfig fileConfig, ErrorReporter errorReporter, LinkedHashMap<String, Object> federationRTIProperties) throws IOException {

    }

    @Override
    public String generateNetworkReceiverBody(Action action, VarRef sendingPort, VarRef receivingPort, FedConnectionInstance connection, InferredType type, CoordinationType coordinationType, ErrorReporter errorReporter) {
        return """
        // generateNetworkReceiverBody
        if (%1$s !== undefined) {
            %2$s.%3$s = %1$s;
        }
        """.formatted(
            action.getName(),
            receivingPort.getContainer().getName(),
            receivingPort.getVariable().getName()
        );
    }

    @Override
    public String generateNetworkSenderBody(VarRef sendingPort, VarRef receivingPort, FedConnectionInstance connection, InferredType type, CoordinationType coordinationType, ErrorReporter errorReporter) {
        return"""
        if (%1$s.%2$s !== undefined) {
            this.util.sendRTITimedMessage(%1$s.%2$s, %3$s, %4$s);
        }
        """.formatted(
            sendingPort.getContainer().getName(),
            sendingPort.getVariable().getName(),
            connection.getDstFederate().id,
            connection.getDstFederate().networkMessageActions.size()
        );
    }

    @Override
    public String generateNetworkInputControlReactionBody(int receivingPortID, TimeValue maxSTP, CoordinationType coordination) {
        return "// TODO(hokeun): Figure out what to do for generateNetworkInputControlReactionBody";
    }

    @Override
    public String generateNetworkOutputControlReactionBody(VarRef srcOutputPort, FedConnectionInstance connection) {
        return "// TODO(hokeun): Figure out what to do for generateNetworkOutputControlReactionBody";
    }

    @Override
    public String getNetworkBufferType() {
        return "";
    }

    /**
     * Add necessary preamble to the source to set up federated execution.
     *
     * @return
     */
    @Override
    public String generatePreamble(FederateInstance federate, FedFileConfig fileConfig, LinkedHashMap<String, Object> federationRTIProperties, ErrorReporter errorReporter) {
        return
        """
        preamble {=
            federated: true,
            id: %d,
            host: %s,
            port: %d,
            network_message_actions: [%s],
            depends_on: [%s],
            sends_to: [%s]
        =}""".formatted(federate.id,
                        federationRTIProperties.get("host"),
                        federationRTIProperties.get("port"),
                        federate.networkMessageActions
                                    .stream()
                                    .map(Variable::getName)
                                    .collect(Collectors.joining(";")),
                        federate.dependsOn.keySet().stream()
                                          .map(e->String.valueOf(e.id))
                                          .collect(Collectors.joining(";")),
                        federate.sendsTo.keySet().stream()
                                        .map(e->String.valueOf(e.id))
                                        .collect(Collectors.joining(";"))
        );
    }
}
