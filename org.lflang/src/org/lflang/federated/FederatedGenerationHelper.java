package org.lflang.federated;

import org.lflang.InferredType;
import org.lflang.TimeValue;
import org.lflang.generator.FederateInstance;
import org.lflang.lf.Action;
import org.lflang.lf.Delay;
import org.lflang.lf.Reaction;
import org.lflang.lf.VarRef;

/**
 * API needed by {@link org.lflang.federated.FedASTUtils}, implemented by generators.
 */
public interface FederatedGenerationHelper {
    // fixme extract those from GeneratorBase? It's too large!
    // fixme this is transitional until we have a single GeneratorBase


    boolean makeUnordered(Reaction reaction);


    int getFederationSize();


    boolean isFederatedAndDecentralized();


    /**
     * Generate code for the body of a reaction that waits long enough so that the status
     * of the trigger for the given port becomes known for the current logical time.
     *
     * @param receivingPortID The port to generate the control reaction for
     * @param maxSTP          The maximum value of STP is assigned to reactions (if any)
     *                        that have port as their trigger or source
     */
    default String generateNetworkInputControlReactionBody(int receivingPortID, TimeValue maxSTP) {
        throw new UnsupportedOperationException("This target does not support direct connections between federates.");
    }


    /**
     * Generate code for the body of a reaction that handles the
     * action that is triggered by receiving a message from a remote
     * federate.
     *
     * @param action                The action.
     * @param sendingPort           The output port providing the data to send.
     * @param receivingPort         The ID of the destination port.
     * @param receivingPortID       The ID of the destination port.
     * @param sendingFed            The sending federate.
     * @param receivingFed          The destination federate.
     * @param receivingBankIndex    The receiving federate's bank index, if it is in a bank.
     * @param receivingChannelIndex The receiving federate's channel index, if it is a multiport.
     * @param type                  The type.
     * @param isPhysical            Indicates whether or not the connection is physical
     */
    default String generateNetworkReceiverBody(
        Action action,
        VarRef sendingPort,
        VarRef receivingPort,
        int receivingPortID,
        FederateInstance sendingFed,
        FederateInstance receivingFed,
        int receivingBankIndex,
        int receivingChannelIndex,
        InferredType type,
        boolean isPhysical
    ) {
        throw new UnsupportedOperationException("This target does not support direct connections between federates.");
    }


    /**
     * Generate code for the body of a reaction that handles an output
     * that is to be sent over the network. This base class throws an exception.
     *
     * @param sendingPort         The output port providing the data to send.
     * @param receivingPort       The ID of the destination port.
     * @param receivingPortID     The ID of the destination port.
     * @param sendingFed          The sending federate.
     * @param sendingBankIndex    The bank index of the sending federate, if it is a bank.
     * @param sendingChannelIndex The channel index of the sending port, if it is a multiport.
     * @param receivingFed        The destination federate.
     * @param type                The type.
     * @param isPhysical          Indicates whether the connection is physical or not
     * @param delay               The delay value imposed on the connection using after
     * @throws UnsupportedOperationException If the target does not support this operation.
     */
    default String generateNetworkSenderBody(
        VarRef sendingPort,
        VarRef receivingPort,
        int receivingPortID,
        FederateInstance sendingFed,
        int sendingBankIndex,
        int sendingChannelIndex,
        FederateInstance receivingFed,
        InferredType type,
        boolean isPhysical,
        Delay delay
    ) {
        throw new UnsupportedOperationException("This target does not support direct connections between federates.");
    }

    /**
     * Generate code for the body of a reaction that sends a port status message for the given
     * port if it is absent.
     *
     * @param port                The port to generate the control reaction for
     * @param portID              The ID assigned to the port in the AST transformation
     * @param receivingFederateID The ID of the receiving federate
     * @param sendingBankIndex    The bank index of the sending federate, if it is a bank.
     * @param sendingChannelIndex The channel if a multiport
     */
    default String generateNetworkOutputControlReactionBody(
        VarRef port,
        int portID,
        int receivingFederateID,
        int sendingBankIndex,
        int sendingChannelIndex
    ) {
        throw new UnsupportedOperationException("This target does not support direct connections between federates.");
    }
}
