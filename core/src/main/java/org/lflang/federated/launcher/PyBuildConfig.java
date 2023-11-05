package org.lflang.federated.launcher;

import org.lflang.MessageReporter;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.generator.FederationFileConfig;

public class PyBuildConfig extends BuildConfig {

  public PyBuildConfig(
      FederateInstance federate, FederationFileConfig fileConfig, MessageReporter messageReporter) {
    super(federate, fileConfig, messageReporter);
  }

  @Override
  public String remoteExecuteCommand() {
    return "bin/" + fileConfig.name + "_" + federate.name + " -i '$FEDERATION_ID'";
  }

  @Override
  public String localExecuteCommand() {
    return fileConfig.getFedBinPath().resolve(federate.name) + " -i $FEDERATION_ID";
  }
}
