package org.lflang.federated.launcher;

import org.lflang.ErrorReporter;
import org.lflang.Target;
import org.lflang.TargetConfig;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;

/**
 * Helper class to get the appropriate launcher generator.
 *
 * FIXME: This architecture needs to be redesigned for multi-target federations.
 */
public class FedLauncherFactory {

    public static FedLauncher getLauncher (
        FederateInstance federate,
        FedFileConfig fileConfig,
        ErrorReporter errorReporter
    ) {
        return getLauncher(Target.fromDecl(federate.target), federate.targetConfig, fileConfig, errorReporter);
    }

    /**
     * Return a launcher generator.
     * @param target        The target to generate for.
     * @param targetConfig  The target config of the federate.
     * @param fileConfig    The file config for the federate.
     * @param errorReporter The error reporter to use.
     * @return null if not supported, an instance of {@code #FedLauncher} otherwise.
     */
    public static FedLauncher getLauncher(
        Target target,
        TargetConfig targetConfig,
        FedFileConfig fileConfig,
        ErrorReporter errorReporter
    ) {
        switch (target) {
        case C:
        case CCPP:
            return new FedCLauncher(
                targetConfig,
                fileConfig,
                errorReporter
            );
        case CPP:
        case Rust:
            return null;
        case TS:
            return new FedTSLauncher(
                targetConfig,
                fileConfig,
                errorReporter
            );
        case Python:
            return new FedPyLauncher(
                targetConfig,
                fileConfig,
                errorReporter
            );
        }
        return null;
    }
}
