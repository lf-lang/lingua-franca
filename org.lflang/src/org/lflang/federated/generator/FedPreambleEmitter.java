package org.lflang.federated.generator;

import static org.lflang.ASTUtils.toText;

import java.io.IOException;
import java.util.LinkedHashMap;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.federated.extensions.FedTargetExtensionFactory;
import org.lflang.federated.launcher.RtiConfig;
import org.lflang.generator.CodeBuilder;
import org.lflang.lf.Model;
import org.lflang.lf.Preamble;

public class FedPreambleEmitter {

    public FedPreambleEmitter() {}

    /**
     * Add necessary code to the source and necessary build support to
     * enable the requested serializations in 'enabledSerializations'
     */
    String generatePreamble(FederateInstance federate, FedFileConfig fileConfig, RtiConfig rtiConfig, ErrorReporter errorReporter)
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

        preambleCode.pr("""
            preamble {=
            %s
            =}""".formatted(FedTargetExtensionFactory.getExtension(federate.targetConfig.target).generatePreamble(
                federate, fileConfig, rtiConfig, errorReporter
            ))
        );

        return preambleCode.getCode();
    }
}
