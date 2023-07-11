package org.lflang.federated.generator;

import java.util.function.Function;
import java.util.stream.Collectors;
import org.eclipse.emf.ecore.EObject;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.ast.FormattingUtil;
import org.lflang.lf.Reactor;

/** Helper class to generate a main reactor */
public class FedMainEmitter {

  /**
   * Generate a main reactor for {@code federate}.
   *
   * @param originalMainReactor The original main reactor.
   * @param messageReporter Used to report errors.
   * @return The main reactor.
   */
  String generateMainReactor(
      FederateInstance federate, Reactor originalMainReactor, MessageReporter messageReporter) {
    // FIXME: Handle modes at the top-level
    if (!ASTUtils.allModes(originalMainReactor).isEmpty()) {
      messageReporter
          .at(ASTUtils.allModes(originalMainReactor).stream().findFirst().orElseThrow())
          .error("Modes at the top level are not supported under federated execution.");
    }
    var renderer = FormattingUtil.renderer(federate.targetConfig.target);

    return String.join(
        "\n",
        generateMainSignature(federate, originalMainReactor, renderer),
        String.join(
                "\n",
                renderer.apply(federate.instantiation),
                ASTUtils.allStateVars(originalMainReactor).stream()
                    .map(renderer)
                    .collect(Collectors.joining("\n")),
                ASTUtils.allActions(originalMainReactor).stream()
                    .filter(federate::includes)
                    .map(renderer)
                    .collect(Collectors.joining("\n")),
                ASTUtils.allTimers(originalMainReactor).stream()
                    .filter(federate::includes)
                    .map(renderer)
                    .collect(Collectors.joining("\n")),
                ASTUtils.allMethods(originalMainReactor).stream()
                    .map(renderer)
                    .collect(Collectors.joining("\n")),
                ASTUtils.allReactions(originalMainReactor).stream()
                    .filter(federate::includes)
                    .map(renderer)
                    .collect(Collectors.joining("\n")),
                federate.networkSenderInstantiations.stream()
                    .map(renderer)
                    .collect(Collectors.joining("\n")),
                federate.networkReceiverInstantiations.stream()
                    .map(renderer)
                    .collect(Collectors.joining("\n")),
                federate.networkHelperInstantiations.stream()
                    .map(renderer)
                    .collect(Collectors.joining("\n")),
                federate.networkConnections.stream()
                    .map(renderer)
                    .collect(Collectors.joining("\n")))
            .indent(4)
            .stripTrailing(),
        "}");
  }

  /**
   * Generate the signature of the main reactor.
   *
   * @param federate The federate.
   * @param originalMainReactor The original main reactor of the original .lf file.
   * @param renderer Used to render EObjects (in String representation).
   */
  private CharSequence generateMainSignature(
      FederateInstance federate, Reactor originalMainReactor, Function<EObject, String> renderer) {
    var paramList =
        ASTUtils.allParameters(originalMainReactor).stream()
            .filter(federate::references)
            .map(renderer)
            .collect(Collectors.joining(",", "(", ")"));

    return """
        @_fed_config()
        main reactor %s {
        """
        .formatted(paramList.equals("()") ? "" : paramList);
  }
}
