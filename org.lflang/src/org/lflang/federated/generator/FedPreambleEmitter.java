package org.lflang.federated.generator;

import static org.lflang.ASTUtils.toText;

import java.io.IOException;
import java.util.LinkedHashMap;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.federated.extensions.FedTargetExtensionFactory;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Model;
import org.lflang.lf.Preamble;

public class FedPreambleEmitter {

    public FedPreambleEmitter() {}

    /**
     * Add necessary code to the source and necessary build support to
     * enable the requested serializations in 'enabledSerializations'
     */
    String generatePreamble(FederateInstance federate, FedFileConfig fileConfig, LinkedHashMap<String, Object> federationRTIProperties, ErrorReporter errorReporter)
        throws IOException {
        CodeBuilder preambleCode = new CodeBuilder();

        // Transfer top-level preambles
        var mainModel = (Model) ASTUtils.toDefinition(federate.instantiation.getReactorClass()).eContainer();
        for (Preamble p : mainModel.getPreambles()) {
            preambleCode.pr(
            """
            %spreamble {=
            %s
            =}
            """.formatted(
                p.getVisibility() == null ? "" : p.getVisibility() + " ",
                toText(p.getCode())
            ));
        }

        preambleCode.pr(FedTargetExtensionFactory.getExtension(federate.target).generatePreamble(
            federate, fileConfig, federationRTIProperties, errorReporter));

        return preambleCode.getCode();
    }
}
