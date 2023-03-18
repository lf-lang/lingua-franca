package org.lflang.federated.launcher;

import org.lflang.ErrorReporter;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;

public abstract class BuildConfig {

    protected final FederateInstance federate;
    protected final ErrorReporter errorReporter;

    protected final FedFileConfig fileConfig;

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
