package org.lflang.federated.extensions;

import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import org.lflang.ErrorReporter;
import org.lflang.FileConfig;
import org.lflang.InferredType;
import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.TimeValue;
import org.lflang.federated.generator.FedConnectionInstance;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.lf.Action;
import org.lflang.lf.Reaction;
import org.lflang.lf.VarRef;

public interface FedTargetExtension {

    /**
     * Perform necessary actions to initialize the target config.
     *
     * @param context
     * @param numOfFederates
     * @param federate       The federate instance.
     * @param fileConfig     An instance of {@code FedFileConfig}.
     * @param errorReporter  Used to report errors.
     */
    void initializeTargetConfig(LFGeneratorContext context, int numOfFederates, FederateInstance federate, FedFileConfig fileConfig,
                                ErrorReporter errorReporter, LinkedHashMap<String, Object> federationRTIProperties) throws IOException;

    /**
     * Generate code for the body of a reaction that handles the
     * action that is triggered by receiving a message from a remote
     * federate.
     * @param action The action.
     * @param sendingPort The output port providing the data to send.
     * @param receivingPort The ID of the destination port.
     * @param connection FIXME
     * @param type FIXME
     * @param coordinationType The coordination type
     * @param errorReporter
     */
    String generateNetworkReceiverBody(
        Action action,
        VarRef sendingPort,
        VarRef receivingPort,
        FedConnectionInstance connection,
        InferredType type,
        CoordinationType coordinationType,
        ErrorReporter errorReporter
    );

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
    String generateNetworkSenderBody(
        VarRef sendingPort,
        VarRef receivingPort,
        FedConnectionInstance connection,
        InferredType type,
        CoordinationType coordinationType,
        ErrorReporter errorReporter
    );

    /**
     * Generate code for the body of a reaction that decides whether the trigger for the given
     * port is going to be present or absent for the current logical time.
     * This reaction is put just before the first reaction that is triggered by the network
     * input port "port" or has it in its sources. If there are only connections to contained
     * reactors, in the top-level reactor.
     *
     * @param receivingPortID The port to generate the control reaction for
     * @param maxSTP The maximum value of STP is assigned to reactions (if any)
     *  that have port as their trigger or source
     * @param coordination FIXME
     */
    String generateNetworkInputControlReactionBody(
        int receivingPortID,
        TimeValue maxSTP,
        CoordinationType coordination
    );

    /**
     * Generate code for the body of a reaction that sends a port status message for the given
     * port if it is absent.
     *
     * @oaram srcOutputPort FIXME
     * @param connection FIXME
     */
    String generateNetworkOutputControlReactionBody(
        VarRef srcOutputPort,
        FedConnectionInstance connection
    );

    /** Optionally apply additional annotations to the reaction. */
    default void annotateReaction(Reaction reaction) {}

    /**
     * Return the type for the raw network buffer in the target language (e.g., `uint_8` in C). This would be the type of the 
     * network messages after serialization and before deserialization. It is primarily used to determine the type for the 
     * network action at the receiver.
     */
    String getNetworkBufferType();

    /**
     * Add necessary preamble to the source to set up federated execution.
     *
     * @param federate
     * @param federationRTIProperties
     * @param errorReporter
     * @return
     */
    String generatePreamble(FederateInstance federate,
                            FedFileConfig fileConfig,
                            LinkedHashMap<String, Object> federationRTIProperties,
                            ErrorReporter errorReporter)
        throws IOException;
}
