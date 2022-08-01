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
                   generateConnections(federate)
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
            federate.getZeroDelayImmediateUpstreamFederates();
        instantiations.pr(zeroDelayImmediateUpstreamFederates
                              .stream()
                              .map(FederateInstance::getInstantiation)
                              .map(inst ->
                                  // FIXME: This is most likely incorrect because
                                  //  bank width and parameters for multiport
                                  //  widths will be lost. However, presence of
                                  //  banks and multiports complicate things
                                  //  tremendously when it comes to connection
                                  //  generation because LF syntax allows for
                                  //  all kinds of intricate connection statements
                                  //  like '(foo.out, bar.out, baz.out)+ -> foo.in'
                                  //  which looks to be quite difficult to untangle.
                                  """
                                  %s = new _lf_%s_interface();
                                  """.formatted(
                                      inst.getName(),
                                      ASTUtils.toDefinition(inst.getReactorClass()).getName()
                                  ))
                              .collect(Collectors.joining("\n")));

        // Then recursively go upstream
        zeroDelayImmediateUpstreamFederates.forEach(federateInstance -> {
            instantiations.pr(generateInstantiationsForUpstreamFederates(federateInstance, renderer));
        });

        return instantiations.getCode();
    }

    /**
     * Generate connection statements that should be in the main reactor for {@code federate}.
     * These connections will help encode the relevant structure of the federation
     * in this federate.
     */
    private CharSequence generateConnections(
        FederateInstance federate
    ) {
        CodeBuilder code = new CodeBuilder();
        var upstreamZeroDelayFederates =
            federate.getZeroDelayImmediateUpstreamFederates();

        for (FederateInstance federateInstance:upstreamZeroDelayFederates) {
            code.pr(generateConnectionsUpstream(federateInstance, new HashSet<>()));
        }
        return code.getCode();
    }

    /**
     * FIXME: For now, use FedConnectionInstances that represent individual connections
     *  between federates to generate connection statements in the main reactor of
     *  {@code federate}. However, since multiport and bank information is lost,
     *  it is not possible to connect two upstream output ports to one downstream
     *  input port.
     */
    private CharSequence generateConnectionsUpstream(
        FederateInstance federate,
        Set<FedConnectionInstance> visited
    ) {
        CodeBuilder code = new CodeBuilder();
        var upstreamZeroDelayFederates =
            federate.getZeroDelayImmediateUpstreamFederates();

        for (FederateInstance federateInstance:upstreamZeroDelayFederates) {
            for (FedConnectionInstance connection : federateInstance.connections) {
                if (visited.contains(connection)) continue;
                visited.add(connection);
                code.pr("""
                %s.%s -> %s.%s;
                """.formatted(
                    connection.srcFederate.getInstantiation().getName(),
                    connection.srcRange.instance.getName(),
                    connection.dstFederate.getInstantiation().getName(),
                    connection.dstRange.instance.getName()
                ));
            }
            code.pr(generateConnectionsUpstream(federateInstance, visited));
        }
        return code.getCode();

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