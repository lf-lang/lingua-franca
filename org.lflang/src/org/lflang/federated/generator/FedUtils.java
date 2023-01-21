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

}
