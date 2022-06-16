package org.lflang.generator.python;

import org.lflang.ErrorReporter;
import org.lflang.TimeValue;
import org.lflang.federated.FedConnectionInstance;
import org.lflang.federated.FederateInstance;
import org.lflang.federated.extensions.FedGeneratorExtension;
import org.lflang.federated.extensions.PythonGeneratorExtension;
import org.lflang.generator.c.CTypes;
import org.lflang.lf.Expression;
import org.lflang.lf.VarRef;
import org.lflang.lf.Action;
import org.lflang.InferredType;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.ReactionInstance;


public class PythonNetworkGenerator implements FedGeneratorExtension {

    @Override
    public String generateNetworkReceiverBody(Action action, VarRef sendingPort, VarRef receivingPort, int receivingPortID, FederateInstance sendingFed, FederateInstance receivingFed, int receivingBankIndex, int receivingChannelIndex, InferredType type, boolean isPhysical, SupportedSerializers serializer, CTypes types, CoordinationType coordinationType) {
        return null;
    }

    @Override
    public String generateNetworkInputControlReactionBody(int receivingPortID, TimeValue maxSTP, boolean isFederatedAndDecentralized) {
        return null;
    }

    @Override
    public String generateNetworkOutputControlReactionBody(VarRef port, int portID, int receivingFederateID, int sendingBankIndex, int sendingChannelIndex, Expression delay) {
        return null;
    }

    /**
     * Generate code for the body of a reaction that handles the
     * action that is triggered by receiving a message from a remote
     * federate.
     * @param action The action.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The ID of the destination port.
     * @param receivingPortID The ID of the destination port.
     * @param sendingFed The sending federate.
     * @param receivingFed The destination federate.
     * @param receivingBankIndex The receiving federate's bank index, if it is in a bank.
     * @param receivingChannelIndex The receiving federate's channel index, if it is a multiport.
     * @param type The type.
     * @param isPhysical Indicates whether or not the connection is physical
     * @param serializer The serializer used on the connection.
     * @param coordinationType The coordination type
     */
    public static String generateNetworkReceiverBody(
        Action action,
        VarRef sendingPort,
        VarRef receivingPort,
        int receivingPortID,
        FederateInstance sendingFed,
        FederateInstance receivingFed,
        int receivingBankIndex,
        int receivingChannelIndex,
        InferredType type,
        boolean isPhysical,
        SupportedSerializers serializer,
        CoordinationType coordinationType
    ) {
        StringBuilder result = new StringBuilder();

        // We currently have no way to mark a reaction "unordered"
        // in the AST, so we use a magic string at the start of the body.
        result.append("// " + ReactionInstance.UNORDERED_REACTION_MARKER + "\n");

        result.append(PyUtil.generateGILAcquireCode() + "\n");
        result.append(PythonGeneratorExtension.generateNetworkReceiverBody(
            action,
            sendingPort,
            receivingPort,
            receivingPortID,
            sendingFed,
            receivingFed,
            receivingBankIndex,
            receivingChannelIndex,
            type,
            isPhysical,
            serializer,
            coordinationType
        ));
        result.append(PyUtil.generateGILReleaseCode() + "\n");
        return result.toString();
    }

    /**
     * Generate code for the body of a reaction that handles an output
     * that is to be sent over the network.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The variable reference to the destination port.
     * @param connection
     * @param type
     * @param coordinationType
     * @param errorReporter FIXME
     */
    public String generateNetworkSenderBody(
        VarRef sendingPort,
        VarRef receivingPort,
        FedConnectionInstance connection,
        InferredType type,
        CoordinationType coordinationType,
        ErrorReporter errorReporter
    ) {
        StringBuilder result = new StringBuilder();

        // We currently have no way to mark a reaction "unordered"
        // in the AST, so we use a magic string at the start of the body.
        result.append("// " + ReactionInstance.UNORDERED_REACTION_MARKER + "\n");

        result.append(PyUtil.generateGILAcquireCode() + "\n");
        result.append(PythonGeneratorExtension.generateNetworkSenderBody(
            sendingPort,
            receivingPort,
            receivingPortID,
            sendingFed,
            sendingBankIndex,
            sendingChannelIndex,
            receivingFed,
            type,
            isPhysical,
            delay,
            serializer,
            coordinationType
        ));
        result.append(PyUtil.generateGILReleaseCode() + "\n");
        return result.toString();
    }
}
