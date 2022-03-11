package org.lflang.generator.python;

import org.lflang.federated.FederateInstance;
import org.lflang.federated.PythonGeneratorExtension;
import org.lflang.lf.VarRef;
import org.lflang.lf.Action;
import org.lflang.lf.Delay;
import org.lflang.InferredType;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.ReactionInstance;


public class PythonNetworkGenerator {
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
        SupportedSerializers serializer
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
            serializer
        ));
        result.append(PyUtil.generateGILReleaseCode() + "\n");
        return result.toString();
    }

    /**
     * Generate code for the body of a reaction that handles an output
     * that is to be sent over the network.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The variable reference to the destination port.
     * @param receivingPortID The ID of the destination port.
     * @param sendingFed The sending federate.
     * @param sendingBankIndex The bank index of the sending federate, if it is a bank.
     * @param sendingChannelIndex The channel index of the sending port, if it is a multiport.
     * @param receivingFed The destination federate.
     * @param type The type.
     * @param isPhysical Indicates whether the connection is physical or not
     * @param delay The delay value imposed on the connection using after
     * @param serializer The serializer used on the connection.
     */
    public static String generateNetworkSenderBody(
        VarRef sendingPort,
        VarRef receivingPort,
        int receivingPortID,
        FederateInstance sendingFed,
        int sendingBankIndex,
        int sendingChannelIndex,
        FederateInstance receivingFed,
        InferredType type,
        boolean isPhysical,
        Delay delay,
        SupportedSerializers serializer,
        CoordinationType coordinationType
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
