package org.lflang.federated.launcher;

import org.lflang.MessageReporter;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.generator.FederationFileConfig;
import org.lflang.generator.c.CCompiler;

/**
 * Utility class that can be used to create a launcher for federated LF programs that are written in
 * C.
 *
 * @author Soroush Bateni
 * @author Marten Lohstroh
 * @ingroup Federated
 */
public class CBuildConfig extends BuildConfig {

  public CBuildConfig(
      FederateInstance federate, FederationFileConfig fileConfig, MessageReporter messageReporter) {
    super(federate, fileConfig, messageReporter);
  }

  @Override
  public String compileCommand() {
    // This generates the compile command to execute remotely via ssh.
    String commandToReturn;
    CCompiler cCompiler = new CCompiler(federate.targetConfig, fileConfig, messageReporter, false);
    commandToReturn = String.join(" ", cCompiler.buildCmakeCommand().toString());
    return commandToReturn;
  }

  @Override
  public String localExecuteCommand() {
    return fileConfig.getFedBinPath().resolve(federate.name) + " -i $FEDERATION_ID";
  }
}
