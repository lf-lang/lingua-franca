package org.lflang.federated.generator;

import java.io.IOException;
import java.util.LinkedHashMap;

import org.lflang.ErrorReporter;
import org.lflang.federated.extensions.FedTargetExtensionFactory;

public class FedPreambleEmitter {

    public FedPreambleEmitter() {}

    /**
     * Add necessary code to the source and necessary build support to
     * enable the requested serializations in 'enabledSerializations'
     */
    String generatePreamble(FederateInstance federate, FedFileConfig fileConfig, LinkedHashMap<String, Object> federationRTIProperties, ErrorReporter errorReporter)
        throws IOException {
        String pre = FedTargetExtensionFactory.getExtension(federate.target).generatePreamble(
            federate, fileConfig, federationRTIProperties, errorReporter);
        if (pre.length() == 0) return "";
        else return
        """
        preamble {=
        %s
        =}""".formatted(pre);
//        if (!IterableExtensions.isNullOrEmpty(enabledSerializers)) {
//            throw new UnsupportedOperationException(
//                "Serialization is target-specific " +
//                    " and is not implemented for the " + " target."
//            );
//        }
    }
}