package org.lflang.federated.generator;

import java.util.LinkedHashMap;

import org.lflang.federated.extensions.FedTargetExtensionFactory;

public class FedPreambleEmitter {

    public FedPreambleEmitter() {}

    /**
     * Add necessary code to the source and necessary build support to
     * enable the requested serializations in 'enabledSerializations'
     */
    String generatePreamble(FederateInstance federate, LinkedHashMap<String, Object> federationRTIProperties) {
        return FedTargetExtensionFactory.getExtension(federate.target).generatePreamble(federate, federationRTIProperties, , , );
//        if (!IterableExtensions.isNullOrEmpty(enabledSerializers)) {
//            throw new UnsupportedOperationException(
//                "Serialization is target-specific " +
//                    " and is not implemented for the " + " target."
//            );
//        }
    }
}