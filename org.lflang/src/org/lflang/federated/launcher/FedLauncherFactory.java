package org.lflang.federated.launcher;

import org.lflang.ErrorReporter;
import org.lflang.Target;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;

/**
 * Helper class to get the appropriate launcher generator.
 *
 * FIXME: This architecture needs to be redesigned for multi-target federations.
 */
public class FedLauncherFactory {

    /**
     * Return a launcher generator.
     * @param federate
     * @param fileConfig
     * @param errorReporter
     * @return null if not supported, an instance of {@code #FedLauncher} otherwise.
     */
    public static FedLauncher getLauncher(
        FederateInstance federate,
        FedFileConfig fileConfig,
        ErrorReporter errorReporter
    ) {
        switch (Target.fromDecl(federate.target)) {
        case C:
        case CCPP:
            return new FedCLauncher(
                federate.targetConfig,
                fileConfig,
                errorReporter
            );
        case CPP:
        case Rust:
            return null;
        case TS:
            return new FedTSLauncher(
                federate.targetConfig,
                fileConfig,
                errorReporter
            );
        case Python:
            return new FedPyLauncher(
                federate.targetConfig,
                fileConfig,
                errorReporter
            );
        }
        return null;
    }
}
