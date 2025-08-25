package org.lflang.federated.launcher;

import org.lflang.MessageReporter;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.generator.FederationFileConfig;

/**
 * Utility class that can be used to create a launcher for federated LF programs that are written in
 * TypeScript.
 *
 * @author Soroush Bateni
 * @author Hokeun Kim
 * @author Marten Lohstroh
 * @ingroup Federated
 */
public class TsBuildConfig extends BuildConfig {

  public TsBuildConfig(
      FederateInstance federate, FederationFileConfig fileConfig, MessageReporter messageReporter) {
    super(federate, fileConfig, messageReporter);
  }

  @Override
  public String compileCommand() {
    return null;
  }

  @Override
  public String localExecuteCommand() {
    String jsFilename = federate.name + ".js";
    return "node "
        + fileConfig.getSrcGenPath().resolve(federate.name).resolve("dist").resolve(jsFilename)
        + " -i $FEDERATION_ID";
  }
}
