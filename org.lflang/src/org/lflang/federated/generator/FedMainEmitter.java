package org.lflang.federated.generator;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.Function;
import java.util.stream.Collectors;

import org.eclipse.emf.ecore.EObject;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.ast.FormattingUtils;
import org.lflang.generator.CodeBuilder;
import org.lflang.generator.ReactorInstance;
import org.lflang.lf.Instantiation;
import org.lflang.lf.Reactor;
import org.lflang.lf.Variable;

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
        var renderer = FormattingUtils.renderer(federate.target);

        return String
            .join(
                "\n",
                generateMainSignature(federate, originalMainReactor, renderer),
               String.join(
                   "\n",
                   renderer.apply(federate.instantiation),
                   ASTUtils.allStateVars(originalMainReactor).stream().filter(federate::contains).map(renderer).collect(Collectors.joining("\n")),
                   ASTUtils.allActions(originalMainReactor).stream().filter(federate::contains).map(renderer).collect(Collectors.joining("\n")),
                   ASTUtils.allTimers(originalMainReactor).stream().filter(federate::contains).map(renderer).collect(Collectors.joining("\n")),
                   ASTUtils.allMethods(originalMainReactor).stream().filter(federate::contains).map(renderer).collect(Collectors.joining("\n")),
                   ASTUtils.allReactions(originalMainReactor).stream().filter(federate::contains).map(renderer).collect(Collectors.joining("\n"))
               ).indent(4).stripTrailing(),
               "}"
            );
    }

    /**
     * Generate the signature of the main reactor.
     * @param federate The federate.
     * @param originalMainReactor The original main reactor of the original .lf file.
     * @param renderer Used to render EObjects (in String representation).
     */
    private CharSequence generateMainSignature(FederateInstance federate, Reactor originalMainReactor, Function<EObject, String> renderer) {
        var paramList = ASTUtils.allParameters(originalMainReactor)
                                .stream()
                                .filter(federate::contains)
                                .map(renderer)
                                .collect(
                                    Collectors.joining(
                                        ",", "(", ")"
                                    )
                                );
        // Empty "()" is currently not allowed by the syntax

        var networkMessageActionsListString = federate.networkMessageActions
            .stream()
            .map(Variable::getName)
            .collect(Collectors.joining(","));

        return
        """
        @_fed_config(network_message_actions="%s")
        main reactor %s {
        """.formatted(networkMessageActionsListString,
                      paramList.equals("()") ? "" : paramList);
    }
}
