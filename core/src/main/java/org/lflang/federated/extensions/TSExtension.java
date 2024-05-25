package org.lflang.federated.extensions;

import static org.lflang.util.StringUtil.addDoubleQuotes;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;
import org.lflang.InferredType;
import org.lflang.MessageReporter;
import org.lflang.TimeValue;
import org.lflang.ast.ASTUtils;
import org.lflang.federated.generator.FedASTUtils;
import org.lflang.federated.generator.FedConnectionInstance;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.generator.FederationFileConfig;
import org.lflang.federated.launcher.RtiConfig;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.ts.TSTypes;
import org.lflang.lf.Action;
import org.lflang.lf.Expression;
import org.lflang.lf.Instantiation;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Output;
import org.lflang.lf.Reactor;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;
import org.lflang.target.property.CoordinationOptionsProperty;
import org.lflang.target.property.CoordinationProperty;
import org.lflang.target.property.type.CoordinationModeType.CoordinationMode;

public class TSExtension implements FedTargetExtension {
  @Override
  public void initializeTargetConfig(
      LFGeneratorContext context,
      List<String> federateNames,
      FederateInstance federate,
      FederationFileConfig fileConfig,
      MessageReporter messageReporter,
      RtiConfig rtiConfig) {}

  @Override
  public String generateNetworkReceiverBody(
      Action action,
      VarRef sendingPort,
      VarRef receivingPort,
      FedConnectionInstance connection,
      InferredType type,
      CoordinationMode coordinationMode,
      MessageReporter messageReporter) {
    return """
        // generateNetworkReceiverBody
        if (%1$s !== undefined) {
            %2$s%3$s = %1$s;
        }
        """
        .formatted(
            action.getName(),
            receivingPort.getContainer() == null
                ? ""
                : receivingPort.getContainer().getName() + ".",
            receivingPort.getVariable().getName());
  }

  @Override
  public String outputInitializationBody() {
    return ""; // TODO
  }

  @Override
  public String inputInitializationBody() {
    return "";
  }

  @Override
  public void addSenderIndexParameter(Reactor sender) {
    var senderIndexParameter = LfFactory.eINSTANCE.createParameter();
    var senderIndexParameterType = LfFactory.eINSTANCE.createType();
    senderIndexParameter.setName("sender_index");
    senderIndexParameterType.setId("Number");
    senderIndexParameter.setType(senderIndexParameterType);
    var senderIndexParameterInit = LfFactory.eINSTANCE.createInitializer();
    var senderIndexParameterInitExpr = LfFactory.eINSTANCE.createLiteral();
    senderIndexParameterInitExpr.setLiteral("0");
    senderIndexParameterInit.setAssign(true);
    senderIndexParameterInit.setExpr(senderIndexParameterInitExpr);
    senderIndexParameter.setInit(senderIndexParameterInit);
    sender.getParameters().add(senderIndexParameter);
  }

  @Override
  public void supplySenderIndexParameter(Instantiation inst, int idx) {
    var senderIndex = LfFactory.eINSTANCE.createAssignment();
    var senderIndexParameter = LfFactory.eINSTANCE.createParameter();
    senderIndexParameter.setName("sender_index");
    senderIndex.setLhs(senderIndexParameter);
    var senderIndexInitializer = LfFactory.eINSTANCE.createInitializer();
    senderIndexInitializer.setAssign(true);
    var senderIndexInitializerExpression = LfFactory.eINSTANCE.createLiteral();
    senderIndexInitializerExpression.setLiteral(String.valueOf(idx));
    senderIndexInitializer.setAssign(true);
    senderIndexInitializer.setExpr(senderIndexInitializerExpression);
    senderIndex.setRhs(senderIndexInitializer);
    inst.getParameters().add(senderIndex);
  }

  @Override
  public String generateNetworkSenderBody(
      VarRef sendingPort,
      VarRef receivingPort,
      FedConnectionInstance connection,
      InferredType type,
      CoordinationMode coordinationMode,
      MessageReporter messageReporter) {
    return """
        if (%1$s%2$s[0] !== undefined) {
            this.util.sendRTITimedMessage(%1$s%2$s[0], %3$s, %4$s, %5$s);
        }
        """
        .formatted(
            sendingPort.getContainer() == null ? "" : sendingPort.getContainer().getName() + ".",
            sendingPort.getVariable().getName(),
            connection.getDstFederate().id,
            connection.getDstFederate().networkMessageActions.size(),
            getNetworkDelayLiteral(connection.getDefinition().getDelay()));
  }

  private String getNetworkDelayLiteral(Expression e) {
    var cLiteral = CExtensionUtils.getNetworkDelayLiteral(e);
    return cLiteral.equals("NEVER") ? "undefined" : "TimeValue.nsec(" + cLiteral + ")";
  }

  @Override
  public String generatePortAbsentReactionBody(
      VarRef srcOutputPort, FedConnectionInstance connection) {
    // The ID of the receiving port (rightPort) is the position
    // of the networkAction (see below) in this list.
    int receivingPortID = connection.getDstFederate().networkMessageActions.size();
    var additionalDelayString = getNetworkDelayLiteral(connection.getDefinition().getDelay());
    return """
        // If the output port has not been set for the current logical time,
        // send an ABSENT message to the receiving federate
        if (%1$s%2$s[0] === undefined) {
          this.util.sendRTIPortAbsent(%3$d, %4$d, %5$s);
        }
      """
        .formatted(
            srcOutputPort.getContainer() == null
                ? ""
                : srcOutputPort.getContainer().getName() + ".",
            srcOutputPort.getVariable().getName(),
            connection.getDstFederate().id,
            receivingPortID,
            additionalDelayString);
  }

  @Override
  public String getNetworkBufferType() {
    return "";
  }

  @Override
  public String generatePreamble(
      FederateInstance federate,
      FederationFileConfig fileConfig,
      RtiConfig rtiConfig,
      MessageReporter messageReporter) {
    var minOutputDelay = getMinOutputDelay(federate, messageReporter);
    var upstreamConnectionDelays = getUpstreamConnectionDelays(federate);
    return """
        const defaultFederateConfig: __FederateConfig = {
            dependsOn: [%s],
            executionTimeout: undefined,
            fast: false,
            federateID: %d,
            federationID: "Unidentified Federation",
            keepAlive: true,
            minOutputDelay: %s,
            networkMessageActions: [%s],
            rtiHost: "%s",
            rtiPort: %d,
            sendsTo: [%s],
            upstreamConnectionDelays: [%s]
        }
            """
        .formatted(
            federate.dependsOn.keySet().stream()
                .map(e -> String.valueOf(e.id))
                .collect(Collectors.joining(",")),
            federate.id,
            minOutputDelay == null
                ? "undefined"
                : "%s".formatted(TSTypes.getInstance().getTargetTimeExpr(minOutputDelay)),
            federate.networkMessageActions.stream()
                .map(Variable::getName)
                .collect(Collectors.joining(",", "\"", "\"")),
            rtiConfig.getHost(),
            rtiConfig.getPort(),
            federate.sendsTo.keySet().stream()
                .map(e -> String.valueOf(e.id))
                .collect(Collectors.joining(",")),
            String.join(",", upstreamConnectionDelays));
  }

  private TimeValue getMinOutputDelay(FederateInstance federate, MessageReporter messageReporter) {
    if (federate.targetConfig.get(CoordinationProperty.INSTANCE) == CoordinationMode.CENTRALIZED) {
      // If this program uses centralized coordination then check
      // for outputs that depend on physical actions so that null messages can be
      // sent to the RTI.
      var federateClass = ASTUtils.toDefinition(federate.instantiation.getReactorClass());
      var main =
          new ReactorInstance(
              FedASTUtils.findFederatedReactor(federate.instantiation.eResource()),
              messageReporter,
              1);
      var instance = new ReactorInstance(federateClass, main, messageReporter);
      var outputDelayMap = federate.findOutputsConnectedToPhysicalActions(instance);
      var minOutputDelay = TimeValue.MAX_VALUE;
      Output outputFound = null;
      for (Output output : outputDelayMap.keySet()) {
        var outputDelay = outputDelayMap.get(output);
        if (outputDelay.isEarlierThan(minOutputDelay)) {
          minOutputDelay = outputDelay;
          outputFound = output;
        }
      }
      if (minOutputDelay != TimeValue.MAX_VALUE) {
        // Unless silenced, issue a warning.
        if (federate.targetConfig.get(CoordinationOptionsProperty.INSTANCE).advanceMessageInterval
            == null) {
          String message =
              String.join(
                  "\n",
                  "Found a path from a physical action to output for reactor "
                      + addDoubleQuotes(instance.getName())
                      + ". ",
                  "The amount of delay is " + minOutputDelay + ".",
                  "With centralized coordination, this can result in a large number of messages to"
                      + " the RTI.",
                  "Consider refactoring the code so that the output does not depend on the physical"
                      + " action,",
                  "or consider using decentralized coordination. To silence this warning, set the"
                      + " target",
                  "parameter coordination-options with a value like {advance-message-interval: 10"
                      + " msec}");
          messageReporter.at(outputFound).warning(message);
        }
        return minOutputDelay;
      }
    }
    return null;
  }

  private List<String> getUpstreamConnectionDelays(FederateInstance federate) {
    List<String> candidates = new ArrayList<>();
    if (!federate.dependsOn.keySet().isEmpty()) {
      for (FederateInstance upstreamFederate : federate.dependsOn.keySet()) {
        StringBuilder element = new StringBuilder("[");
        var delays = federate.dependsOn.get(upstreamFederate);
        int cnt = 0;
        if (delays != null) {
          for (Expression delay : delays) {
            if (delay == null) {
              element.append("TimeValue.never()");
            } else {
              element.append(getNetworkDelayLiteral(delay));
            }
            cnt++;
            if (cnt != delays.size()) {
              element.append(", ");
            }
          }
        } else {
          element.append("TimeValue.never()");
        }
        element.append("]");
        candidates.add(element.toString());
      }
    }
    return candidates;
  }
}
