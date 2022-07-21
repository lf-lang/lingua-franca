package org.lflang.federated.generator;

import static org.lflang.ASTUtils.convertToEmptyListIfNull;

import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.stream.Collectors;

import org.lflang.ErrorReporter;
import org.lflang.Target;
import org.lflang.TargetProperty;
import org.lflang.ast.ToLf;
import org.lflang.federated.extensions.FedTargetExtensionFactory;
import org.lflang.generator.GeneratorUtils;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.lf.ImportedReactor;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Reactor;

public class FedTargetEmitter {

    String generateTarget(
        LFGeneratorContext context,
        FederateInstance federate,
        FedFileConfig fileConfig,
        ErrorReporter errorReporter,
        LinkedHashMap<String, Object> federationRTIProperties
    ) throws IOException {

        GeneratorUtils.setTargetConfig(
            context,
            federate.target,
            federate.targetConfig,
            errorReporter
        );
        // FIXME: Should we merge some properties with the main .lf file if the federate is imported?
        // Maybe only if their targets match?
        var fedReactorClass = federate.instantiation.getReactorClass();
        if (!fedReactorClass.eResource().equals(fileConfig.resource)) {
            // Merge some target properties of the main .lf file.
            var target = GeneratorUtils.findTarget(fileConfig.resource);
            if (target.getConfig() != null) {
                // Merge properties
                TargetProperty.update(
                    federate.targetConfig,
                    convertToEmptyListIfNull(target.getConfig().getPairs()),
                    errorReporter
                );
            }
        }

        FedTargetExtensionFactory.getExtension(federate.target)
                                 .initializeTargetConfig(context, federate, fileConfig, errorReporter, federationRTIProperties);

        return ToLf.instance.doSwitch(
            TargetProperty.extractTargetDecl(
                Target.fromDecl(federate.target),
                federate.targetConfig
            )
        );
    }
}