package org.lflang.federated.extensions;

import org.lflang.ErrorReporter;
import org.lflang.InferredType;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.TimeValue;
import org.lflang.federated.generator.FedConnectionInstance;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.lf.Action;
import org.lflang.lf.VarRef;

public class CppExtension implements FedTargetExtension {

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
    public String generatePreamble(FederateInstance federate) {
        return
        """
        public preamble {=
            %s
        =}""".formatted("");
    }
}
