package org.lflang.federated.extensions;

import java.io.IOException;
import java.util.LinkedHashMap;

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

public class RustExtension implements FedTargetExtension {

    @Override
    public void initializeTargetConfig(FederateInstance federate, FedFileConfig fileConfig, ErrorReporter errorReporter, LinkedHashMap<String, Object> federationRTIProperties) throws IOException {

    }

    @Override
    public String generateNetworkReceiverBody(Action action, VarRef sendingPort, VarRef receivingPort, FedConnectionInstance connection, InferredType type, CoordinationType coordinationType, ErrorReporter errorReporter) {
        return null;
    }

    @Override
    public String generateNetworkSenderBody(VarRef sendingPort, VarRef receivingPort, FedConnectionInstance connection, InferredType type, CoordinationType coordinationType, ErrorReporter errorReporter) {
        return null;
    }

    @Override
    public String generateNetworkInputControlReactionBody(int receivingPortID, TimeValue maxSTP, CoordinationType coordination) {
        return null;
    }

    @Override
    public String generateNetworkOutputControlReactionBody(VarRef srcOutputPort, FedConnectionInstance connection) {
        return null;
    }

    @Override
    public String getNetworkBufferType() {
        return null;
    }

    /**
     * Add necessary preamble to the source to set up federated execution.
     *
     * @return
     */
    @Override
    public String generatePreamble(FederateInstance federate, LinkedHashMap<String, Object> federationRTIProperties) {
        return
        """
        preamble {=
            %s
        =}""".formatted("");
    }
}
