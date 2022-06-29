package org.lflang.federated.generator;

import java.util.stream.Collectors;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.ast.ToLf;
import org.lflang.lf.Reactor;

/**
 * Helper class to generate a main reactor
 */
public class FedMainEmitter {

    /**
     * Generate a main reactor for {@code federate}.
     *
     * @param federate
     * @param originalMainReactor The original main reactor.
     * @param errorReporter       Used to report errors.
     * @return The main reactor.
     */
    String generateMainReactor(FederateInstance federate, Reactor originalMainReactor, ErrorReporter errorReporter) {
        // FIXME: Handle modes at the top-level
        if (!ASTUtils.allModes(originalMainReactor).isEmpty()) {
            errorReporter.reportError(
                ASTUtils.allModes(originalMainReactor).stream().findFirst().get(),
                "Modes at the top level are not supported under federated execution."
            );
        }

        return String
            .join(
                "\n",
               "main reactor {",
               "    "+ToLf.instance.doSwitch(federate.instantiation),
               "    "+ASTUtils.allActions(originalMainReactor).stream().filter(federate::contains).map(ToLf.instance::doSwitch).collect(Collectors.joining("\n")),
               "    "+ASTUtils.allTimers(originalMainReactor).stream().filter(federate::contains).map(ToLf.instance::doSwitch).collect(Collectors.joining("\n")),
               "    "+ASTUtils.allMethods(originalMainReactor).stream().filter(federate::contains).map(ToLf.instance::doSwitch).collect(Collectors.joining("\n")),
               "    "+ASTUtils.allReactions(originalMainReactor).stream().filter(federate::contains).map(ToLf.instance::doSwitch).collect(Collectors.joining("\n")),
               "}"
        );
    }
}