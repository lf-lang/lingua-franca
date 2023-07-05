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
  public String localExecuteCommand() {
    return "python3 "
        + fileConfig.getSrcGenPath()
        + "/"
        + federate.name
        + "/"
        + federate.name
        + ".py -i $FEDERATION_ID";
  }

  @Override
  public String remoteExecuteCommand() {
    return "python3 src-gen/"
        + fileConfig.name
        + "/"
        + federate.name
        + "/"
        + fileConfig.name
        + "_"
        + federate.name
        + " -i '$FEDERATION_ID'";
  }
}
