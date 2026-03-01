package org.lflang.federated.generator;

import java.util.function.Function;
import java.util.stream.Collectors;
import org.eclipse.emf.ecore.EObject;
import org.eclipse.emf.ecore.util.EcoreUtil;
import org.lflang.MessageReporter;
import org.lflang.ast.ASTUtils;
import org.lflang.ast.FormattingUtil;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Reactor;

/**
 * Helper class to generate a main reactor.
 *
 * @ingroup Federated
 */
public class FedMainEmitter {

  /**
   * Generate a main reactor for `federate`.
   *
   * @param federate The federate instance.
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
    var instantiation = EcoreUtil.copy(federate.instantiation);
    instantiation.setWidthSpec(null);
    if (federate.bankWidth > 1) {
      var assignment = LfFactory.eINSTANCE.createAssignment();
      var parameter = LfFactory.eINSTANCE.createParameter();
      parameter.setName("bank_index");
      assignment.setLhs(parameter);
      var initializer = LfFactory.eINSTANCE.createInitializer();
      var expression = LfFactory.eINSTANCE.createLiteral();
      expression.setLiteral(String.valueOf(federate.bankIndex));
      initializer.setAssign(true);
      initializer.setExpr(expression);
      assignment.setRhs(initializer);
      instantiation.getParameters().add(assignment);
    }

    return String.join(
        "\n",
        generateMainSignature(federate, originalMainReactor, renderer),
        String.join(
                "\n",
                renderer.apply(instantiation),
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
