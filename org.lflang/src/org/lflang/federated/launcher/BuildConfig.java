package org.lflang.federated.launcher;

import org.lflang.ErrorReporter;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;

/**
 * A collection of methods used for building target code for federates.
 */
public abstract class BuildConfig {

    /**
     * The federate that this configuration applies to.
     */
    protected final FederateInstance federate;

    /**
     * An error reporter to report problems.
     */
    protected final ErrorReporter errorReporter;

    /**
     * The file configuration of the federation that the federate belongs to.
     */
    protected final FedFileConfig fileConfig;

    /**
     * Create a new build configuration.
     *
     * @param federate The federate that this configuration applies to.
     * @param fileConfig The file configuration of the federation that the federate belongs to.
     * @param errorReporter An error reporter to report problems.
     */
    public BuildConfig(FederateInstance federate, FedFileConfig fileConfig, ErrorReporter errorReporter) {
        this.errorReporter = errorReporter;
        this.federate = federate;
        this.fileConfig = fileConfig;
    }

    /**
     * Return the compile command for the federate that this build configuration belongs to.
     */
    public String compileCommand() {
        throw new UnsupportedOperationException();
    }

    /**
     * Return the command that will execute the federate that this build configuration belongs to
     * locally, assuming that the current directory is the top-level project folder.
     */
    public abstract String localExecuteCommand();

    /**
     * Return the command that will execute the federate that this build configuration belongs to
     * remotely, assuming that the current directory is the top-level project folder.
     */
    public String remoteExecuteCommand() {
        throw new UnsupportedOperationException();
    }
}
