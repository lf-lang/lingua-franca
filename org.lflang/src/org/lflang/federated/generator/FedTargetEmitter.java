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
import org.lflang.federated.launcher.RtiConfig;
import org.lflang.generator.GeneratorUtils;
import org.lflang.generator.LFGeneratorContext;

public class FedTargetEmitter {

    String generateTarget(
        LFGeneratorContext context,
        int numOfFederates,
        FederateInstance federate,
        FedFileConfig fileConfig,
        ErrorReporter errorReporter,
        RtiConfig rtiConfig
    ) throws IOException {

        // FIXME: First of all, this is not an initialization; there is all sorts of stuff happening
        // in the implementations of this method. Second, true initialization stuff should happen
        // when the target config is constructed, not when we're doing code generation.
        FedTargetExtensionFactory.getExtension(federate.targetConfig.target)
                                 .initializeTargetConfig(
                                     context,
                                     numOfFederates,
                                     federate,
                                     fileConfig,
                                     errorReporter,
                                     rtiConfig
                                 );

        return FormattingUtils.renderer(federate.targetConfig.target).apply(
            TargetProperty.extractTargetDecl(
                federate.targetConfig.target,
                federate.targetConfig
            )
        );
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
