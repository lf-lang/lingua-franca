package org.lflang.federated.extensions;

import java.io.IOException;

import org.lflang.ErrorReporter;
import org.lflang.InferredType;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.TimeValue;
import org.lflang.federated.generator.FedConnectionInstance;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.lf.Action;
import org.lflang.lf.VarRef;

public class TSExtension implements FedTargetExtension {

    @Override
    public void initializeTargetConfig(FederateInstance federate, FedFileConfig fileConfig, TargetConfig targetConfig, ErrorReporter errorReporter) throws IOException {

    }

    @Override
    public String generateNetworkReceiverBody(Action action, VarRef sendingPort, VarRef receivingPort, FedConnectionInstance connection, InferredType type, CoordinationType coordinationType, ErrorReporter errorReporter) {

        return """
        // generateNetworkReceiverBody
        if (%1$s !== undefined) {
            %s.%s = %1$s;
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
            this.util.sendRTITimedMessage(%1$s.%2$s, %s, %s);
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
    public String generatePreamble(FederateInstance federate) {
//        for (serializer in enabledSerializers) {
//            when (serializer) {
//                SupportedSerializers.NATIVE -> {
//                    // No need to do anything at this point.
//                    println("Native serializer is enabled.")
//                }
//                else -> throw UnsupportedOperationException("Unsupported serializer: $serializer");
//            }
//        }
        return
        """
        preamble {=
            %s
        =}""".formatted("");
    }
}
