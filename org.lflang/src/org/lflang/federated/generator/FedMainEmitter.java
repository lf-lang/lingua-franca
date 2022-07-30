package org.lflang.federated.generator;

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
        var renderer = FormattingUtils.renderer(federate.target);

        return String
            .join(
                "\n",
                generateMainSignature(federate, originalMainReactor, renderer),
               String.join(
                   "\n",
                   renderer.apply(federate.instantiation),
                   generateInstantiationsForUpstreamFederates(federate, renderer),
                   ASTUtils.allActions(originalMainReactor).stream().filter(federate::contains).map(renderer).collect(Collectors.joining("\n")),
                   ASTUtils.allTimers(originalMainReactor).stream().filter(federate::contains).map(renderer).collect(Collectors.joining("\n")),
                   ASTUtils.allMethods(originalMainReactor).stream().filter(federate::contains).map(renderer).collect(Collectors.joining("\n")),
                   ASTUtils.allReactions(originalMainReactor).stream().filter(federate::contains).map(renderer).collect(Collectors.joining("\n")),
                   generateConnections(federate, originalMainReactor, renderer)
               ).indent(4).stripTrailing(),
               "}"
            );
    }

    /**
     * Generate instantiations for upstream federates of {@code federate}.
     * @param federate
     * @param renderer Used to generate a String representation for an instantiation.
     * @return
     */
    private CharSequence generateInstantiationsForUpstreamFederates(FederateInstance federate, Function<EObject, String> renderer) {
        CodeBuilder instantiations = new CodeBuilder();
        // First handle immediate upstream federates with 0 delays
        var zeroDelayImmediateUpstreamFederates =
            federate.dependsOn.entrySet()
                              .stream()
                              .filter(e -> e.getValue().contains(null))
                              .map(Map.Entry::getKey).toList();
        instantiations.pr(zeroDelayImmediateUpstreamFederates
                              .stream()
                              .map(FederateInstance::getInstantiation)
                              .map(renderer)
                              .collect(Collectors.joining("\n")));

        // Then recursively go upstream
        zeroDelayImmediateUpstreamFederates.forEach(federateInstance -> {
            instantiations.pr(generateInstantiationsForUpstreamFederates(federateInstance, renderer));
        });

        return instantiations.getCode();
    }

    /**
     * Generate connections.
     * @param federate
     * @param originalMainReactor
     * @param renderer
     * @return
     */
    private CharSequence generateConnections(FederateInstance federate, Reactor originalMainReactor, Function<EObject, String> renderer) {
        return ASTUtils.allConnections(originalMainReactor).stream().map(
                    connection -> {
                        connection.getLeftPorts().removeIf(
                          varRef -> varRef.getContainer()
                                          .equals(federate.instantiation)
                        );
                        connection.getRightPorts().removeIf(
                            varRef -> varRef.getContainer()
                                            .equals(federate.instantiation)

                        );
                        if (connection.getLeftPorts().isEmpty() || connection.getRightPorts().isEmpty()) {
                            return null;
                        } else {
                            return connection;
                        }
                    }
                ).filter(Objects::nonNull).map(renderer).collect(Collectors.joining("\n"));
    }

    /**
     * Generate the signature of the main reactor.
     * @param federate
     * @param originalMainReactor
     * @param renderer
     * @return
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
        return
        """
        main reactor %s {
        """.formatted(paramList.equals("()") ? "" : paramList);
    }
}