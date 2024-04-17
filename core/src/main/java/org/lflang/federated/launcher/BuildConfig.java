package org.lflang.federated.launcher;

import org.lflang.MessageReporter;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.generator.FederationFileConfig;

/** A collection of methods used for building target code for federates. */
public abstract class BuildConfig {

  /** The federate that this configuration applies to. */
  protected final FederateInstance federate;

  /** An error reporter to report problems. */
  protected final MessageReporter messageReporter;

  /** The file configuration of the federation that the federate belongs to. */
  protected final FederationFileConfig fileConfig;

  /**
   * Create a new build configuration.
   *
   * @param federate The federate that this configuration applies to.
   * @param fileConfig The file configuration of the federation that the federate belongs to.
   * @param messageReporter An error reporter to report problems.
   */
  public BuildConfig(
      FederateInstance federate, FederationFileConfig fileConfig, MessageReporter messageReporter) {
    this.messageReporter = messageReporter;
    this.federate = federate;
    this.fileConfig = fileConfig;
  }

  /** Return the compile command for the federate that this build configuration belongs to. */
  public String compileCommand() {
    throw new UnsupportedOperationException();
  }

  /**
   * Return the command that will execute the federate that this build configuration belongs to
   * locally, assuming that the current directory is the top-level project folder.
   */
  public abstract String localExecuteCommand();
}
