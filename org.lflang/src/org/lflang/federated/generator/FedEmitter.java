package org.lflang.federated.generator;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import org.lflang.ErrorReporter;
import org.lflang.generator.CodeMap;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.lf.Reactor;

/**
 * Helper class to generate code for federates.
 */
public class FedEmitter {

    private final FedFileConfig fileConfig;
    private final Reactor originalMainReactor;
    private final ErrorReporter errorReporter;
    private final LinkedHashMap<String, Object> federationRTIProperties;

    public FedEmitter(
        FedFileConfig fileConfig,
        Reactor originalMainReactor,
        ErrorReporter errorReporter,
        LinkedHashMap<String, Object> federationRTIProperties
    ) {
        this.fileConfig = fileConfig;
        this.originalMainReactor = originalMainReactor;
        this.errorReporter = errorReporter;
        this.federationRTIProperties = federationRTIProperties;
    }

    /**
     * Generate a .lf file for federate {@code federate}.
     *
     * @throws IOException
     */
    Map<Path, CodeMap> generateFederate(
        LFGeneratorContext context,
        FederateInstance federate,
        int numOfFederates
    ) throws IOException {
        String fedName = federate.name;
        Files.createDirectories(fileConfig.getSrcPath());
        System.out.println("##### Generating code for federate " + fedName
                               + " in directory "
                               + fileConfig.getSrcPath());

        Path lfFilePath = fileConfig.getSrcPath().resolve(
            fedName + ".lf");

        String federateCode = String.join(
            "\n",
            new FedTargetEmitter().generateTarget(context, numOfFederates, federate, fileConfig, errorReporter, federationRTIProperties),
            new FedImportEmitter().generateImports(federate, fileConfig),
            new FedPreambleEmitter().generatePreamble(federate, fileConfig, federationRTIProperties, errorReporter),
            new FedReactorEmitter().generateReactorDefinitions(federate),
            new FedMainEmitter().generateMainReactor(
                federate,
                originalMainReactor,
                errorReporter
            )
        );
        Map<Path, CodeMap> codeMapMap = new HashMap<>();
        try (var srcWriter = Files.newBufferedWriter(lfFilePath)) {
            var codeMap = CodeMap.fromGeneratedCode(federateCode);
            codeMapMap.put(lfFilePath, codeMap);
            srcWriter.write(codeMap.getGeneratedCode());
        }
        return codeMapMap;
    }
}
