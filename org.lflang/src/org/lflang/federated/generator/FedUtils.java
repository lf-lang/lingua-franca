package org.lflang.federated.generator;

import java.util.List;

import org.lflang.federated.serialization.SupportedSerializers;
import org.lflang.generator.PortInstance;
import org.lflang.generator.ReactionInstance;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Connection;
import org.lflang.lf.Reaction;
import org.lflang.lf.VarRef;

/**
 * A collection of utility methods for the federated generator.
 */
public class FedUtils {
    /**
     * Get the serializer for the {@code connection} between {@code srcFederate} and {@code dstFederate}.
     */
    public static SupportedSerializers getSerializer(
        Connection connection,
        FederateInstance srcFederate,
        FederateInstance dstFederate
    ) {
        // Get the serializer
        SupportedSerializers serializer = SupportedSerializers.NATIVE;
        if (connection.getSerializer() != null) {
            serializer = SupportedSerializers.valueOf(
                connection.getSerializer().getType().toUpperCase()
            );
        }
        // Add it to the list of enabled serializers for the source and destination federates
        srcFederate.enabledSerializers.add(serializer);
        dstFederate.enabledSerializers.add(serializer);
        return serializer;
    }



    /**
     * Remove triggers in each federates' network reactions that are defined
     * in remote federates.
     *
     * This must be done in code generators after the dependency graphs
     * are built and levels are assigned. Otherwise, these disconnected ports
     * might reference data structures in remote federates and cause
     * compile/runtime errors.
     *
     * @param instance The reactor instance to remove these ports from if any.
     *  Can be null.
     */
    protected void removeRemoteFederateConnectionPorts(ReactorInstance instance, List<FederateInstance> federates ) {
        for (FederateInstance federate : federates) {
            // Remove disconnected network triggers from the AST
            federate.removeRemoteFederateConnectionPorts();
            if (instance == null) {
                continue;
            }
            // If passed a reactor instance, also purge the disconnected network triggers
            // from the reactor instance graph
            for (Reaction reaction : federate.networkReactions) {
                ReactionInstance networkReaction = instance.lookupReactionInstance(reaction);
                if (networkReaction == null) {
                    continue;
                }
                for (VarRef port : federate.remoteNetworkReactionTriggers) {
                    PortInstance disconnectedPortInstance = instance.lookupPortInstance(port);
                    if (disconnectedPortInstance != null) {
                        networkReaction.removePortInstance(disconnectedPortInstance);
                    }
                }
            }
        }
    }
}
