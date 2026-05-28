package org.lflang.federated.launcher;

import org.lflang.MessageReporter;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.generator.FederationFileConfig;
import org.lflang.federated.generator.SSTGenerator;
import org.lflang.federated.generator.TLSGenerator;
import org.lflang.target.property.CommunicationModeProperty;
import org.lflang.target.property.type.CommunicationModeType.CommunicationMode;

/**
 * A build configuration for Python federates.
 *
 * @ingroup Federated
 */
public class PyBuildConfig extends BuildConfig {

  public PyBuildConfig(
      FederateInstance federate, FederationFileConfig fileConfig, MessageReporter messageReporter) {
    super(federate, fileConfig, messageReporter);
  }

  @Override
  public String localExecuteCommand() {
    String commandToReturn =
        fileConfig.getFedBinPath().resolve(federate.name) + " -i $FEDERATION_ID";
    if (federate.targetConfig.get(CommunicationModeProperty.INSTANCE) == CommunicationMode.SST) {
      commandToReturn =
          commandToReturn
              + " -sst "
              + SSTGenerator.getSSTConfig(fileConfig, federate.name).toString();
    } else if (federate.targetConfig.get(CommunicationModeProperty.INSTANCE)
        == CommunicationMode.TLS) {
      commandToReturn +=
          " -tls "
              + TLSGenerator.getLocalCertPath(fileConfig, federate.name).toString()
              + " "
              + TLSGenerator.getLocalKeyPath(fileConfig, federate.name).toString();
    }
    return commandToReturn;
  }
}
