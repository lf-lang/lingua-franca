package org.lflang.federated.generator;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.HashMap;
import java.util.Map;
import org.lflang.MessageReporter;
import org.lflang.federated.launcher.RtiConfig;
import org.lflang.generator.CodeMap;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.lf.Reactor;

/** Helper class to generate code for federates. */
public class FedEmitter {

  private final FedFileConfig fileConfig;
  private final Reactor originalMainReactor;
  private final MessageReporter messageReporter;
  private final RtiConfig rtiConfig;

  public FedEmitter(
      FedFileConfig fileConfig,
      Reactor originalMainReactor,
      MessageReporter messageReporter,
      RtiConfig rtiConfig) {
    this.fileConfig = fileConfig;
    this.originalMainReactor = originalMainReactor;
    this.messageReporter = messageReporter;
    this.rtiConfig = rtiConfig;
  }

  /**
   * Generate a .lf file for federate {@code federate}.
   *
   * @throws IOException
   */
  Map<Path, CodeMap> generateFederate(
      LFGeneratorContext context, FederateInstance federate, int numOfFederates)
      throws IOException {
    String fedName = federate.name;
    Files.createDirectories(fileConfig.getSrcPath());
    messageReporter
        .nowhere()
        .info(
            "##### Generating code for federate "
                + fedName
                + " in directory "
                + fileConfig.getSrcPath());

    String federateCode =
        String.join(
            "\n",
            new FedTargetEmitter()
                .generateTarget(
                    context, numOfFederates, federate, fileConfig, messageReporter, rtiConfig),
            new FedImportEmitter().generateImports(federate, fileConfig),
            new FedPreambleEmitter()
                .generatePreamble(federate, fileConfig, rtiConfig, messageReporter),
            new FedReactorEmitter().generateReactorDefinitions(federate),
            new FedMainEmitter()
                .generateMainReactor(federate, originalMainReactor, messageReporter));
    Map<Path, CodeMap> codeMapMap = new HashMap<>();
    var lfFilePath = lfFilePath(fileConfig, federate);
    try (var srcWriter = Files.newBufferedWriter(lfFilePath)) {
      var codeMap = CodeMap.fromGeneratedCode(federateCode);
      codeMapMap.put(lfFilePath, codeMap);
      srcWriter.write(codeMap.getGeneratedCode());
    }
    return codeMapMap;
  }

  public static Path lfFilePath(FedFileConfig fileConfig, FederateInstance federate) {
    return fileConfig.getSrcPath().resolve(federate.name + ".lf");
  }
}
