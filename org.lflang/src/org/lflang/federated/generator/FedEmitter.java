package org.lflang.federated.generator;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty;
import org.lflang.federated.extensions.CExtensionUtils;
import org.lflang.federated.extensions.FedTargetExtensionFactory;
import org.lflang.generator.GeneratorUtils;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.Reactor;

/**
 * Helper class to generate code for federates.
 */
public class FedEmitter {

    private final FedFileConfig fileConfig;
    private final Reactor originalMainReactor;
    private final ErrorReporter errorReporter;

    public FedEmitter(
        FedFileConfig fileConfig,
        Reactor originalMainReactor,
        ErrorReporter errorReporter
    ) {
        this.fileConfig = fileConfig;
        this.originalMainReactor = originalMainReactor;
        this.errorReporter = errorReporter;
    }

    /**
     * Generate a .lf file for federate {@code federate}.
     *
     * @throws IOException
     */
    void generateFederate(FederateInstance federate) throws IOException {
        String fedName = federate.instantiation.getName();
        Files.createDirectories(fileConfig.getFedSrcPath());
        System.out.println("##### Generating code for federate " + fedName
                               + " in directory "
                               + fileConfig.getFedSrcPath());

        Path lfFilePath = fileConfig.getFedSrcPath().resolve(
            fedName + ".lf");

        String federateCode = String.join(
            "\n",
            (new FedTargetEmitter()).generateTarget(federate, fileConfig, errorReporter),
            (new FedImportEmitter()).generateImports(federate, fileConfig),
            (new FedPreambleEmitter()).generatePreamble(federate),
            (new FedReactorEmitter()).generateReactorDefinitions(federate),
            (new FedMainEmitter()).generateMainReactor(
                federate,
                originalMainReactor,
                errorReporter
            )
        );

        try (var srcWriter = Files.newBufferedWriter(lfFilePath)) {
            srcWriter.write(federateCode);
        }

    }
}