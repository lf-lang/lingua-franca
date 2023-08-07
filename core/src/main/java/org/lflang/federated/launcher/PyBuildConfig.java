package org.lflang.federated.launcher;

import org.lflang.MessageReporter;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;

public class PyBuildConfig extends BuildConfig {

  public PyBuildConfig(
      FederateInstance federate, FedFileConfig fileConfig, MessageReporter messageReporter) {
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
