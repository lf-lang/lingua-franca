package org.lflang.federated.generator;

import static org.lflang.ASTUtils.convertToEmptyListIfNull;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import org.lflang.ErrorReporter;
import org.lflang.Target;
import org.lflang.TargetProperty;
import org.lflang.ast.FormattingUtils;
import org.lflang.federated.extensions.FedTargetExtensionFactory;
import org.lflang.generator.GeneratorUtils;
import org.lflang.generator.LFGeneratorContext;

public class FedTargetEmitter {

    String generateTarget(
        LFGeneratorContext context,
        int numOfFederates,
        FederateInstance federate,
        FedFileConfig fileConfig,
        ErrorReporter errorReporter,
        LinkedHashMap<String, Object> federationRTIProperties
    ) throws IOException {
        federate.targetConfig =
        GeneratorUtils.getTargetConfig(
            context.getArgs(),
            federate.target,
            errorReporter
        );
        // FIXME: Should we merge some properties with the main .lf file if the federate is imported?
        // https://issue.lf-lang.org/1560
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

        relativizeTargetPaths(federate, fileConfig);

        clearFederatedTargetPropertiesI(federate);

        FedTargetExtensionFactory.getExtension(federate.target)
                                 .initializeTargetConfig(
                                     context,
                                     numOfFederates,
                                     federate,
                                     fileConfig,
                                     errorReporter,
                                     federationRTIProperties
                                 );

        return FormattingUtils.renderer(federate.target).apply(
            TargetProperty.extractTargetDecl(
                Target.fromDecl(federate.target),
                federate.targetConfig
            )
        );
    }

    /**
     * Clear target properties that should not end up in the generated .lf file
     * for {@code federate}.
     */
    private void clearFederatedTargetPropertiesI(FederateInstance federate) {
        federate.targetConfig.setByUser.remove(TargetProperty.CLOCK_SYNC);
        federate.targetConfig.setByUser.remove(TargetProperty.CLOCK_SYNC_OPTIONS);
    }


    /**
     * Relativize target properties that involve paths like files and cmake-include to be
     * relative to the generated .lf file for the federate.
     */
    private void relativizeTargetPaths(FederateInstance federate, FedFileConfig fileConfig) {
        // FIXME: Should we relativize here or calculate absolute paths?
        relativizePathList(federate.targetConfig.protoFiles, fileConfig);

        relativizePathList(federate.targetConfig.fileNames, fileConfig);

        relativizePathList(federate.targetConfig.cmakeIncludes, fileConfig);
    }

    private void relativizePathList(List<String> paths, FedFileConfig fileConfig) {
        List<String> tempList = new ArrayList<>();
        paths.forEach( f -> {
            tempList.add(relativizePath(f, fileConfig));
        });
        paths.clear();
        paths.addAll(tempList);
    }

    private String relativizePath(String path, FedFileConfig fileConfig) {
        Path resolvedPath = fileConfig.srcPath.resolve(path).toAbsolutePath();
        return fileConfig.getSrcPath().relativize(resolvedPath).toString();
    }
}
