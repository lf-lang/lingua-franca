package org.lflang.federated.extensions;

import static org.lflang.util.StringUtil.addDoubleQuotes;

import java.io.IOException;
import java.util.LinkedHashMap;
import java.util.stream.Collectors;

import org.lflang.ASTUtils;
import org.lflang.ErrorReporter;
import org.lflang.InferredType;
import org.lflang.Target;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.TimeValue;
import org.lflang.federated.generator.FedASTUtils;
import org.lflang.federated.generator.FedConnectionInstance;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.generator.GeneratorBase;
import org.lflang.generator.LFGeneratorContext;
import org.lflang.generator.ReactorInstance;
import org.lflang.generator.ts.TSExtensionsKt;
import org.lflang.lf.Action;
import org.lflang.lf.Expression;
import org.lflang.lf.Output;
import org.lflang.lf.VarRef;
import org.lflang.lf.Variable;

public class TSExtension implements FedTargetExtension {
    @Override
    public void initializeTargetConfig(LFGeneratorContext context, int numOfFederates, FederateInstance federate, FedFileConfig fileConfig, ErrorReporter errorReporter, LinkedHashMap<String, Object> federationRTIProperties) throws IOException {

    }

    @Override
    public String generateNetworkReceiverBody(Action action, VarRef sendingPort, VarRef receivingPort, FedConnectionInstance connection, InferredType type, CoordinationType coordinationType, ErrorReporter errorReporter) {
        return """
        // generateNetworkReceiverBody
        if (%1$s !== undefined) {
            %2$s.%3$s = %1$s;
        }
        """.formatted(
            action.getName(),
            receivingPort.getContainer().getName(),
            receivingPort.getVariable().getName()
        );
    }

    @Override
    public String generateNetworkSenderBody(VarRef sendingPort, VarRef receivingPort, FedConnectionInstance connection, InferredType type, CoordinationType coordinationType, ErrorReporter errorReporter) {
        return"""
        if (%1$s.%2$s !== undefined) {
            this.util.sendRTITimedMessage(%1$s.%2$s, %3$s, %4$s, %5$s);
        }
        """.formatted(
            sendingPort.getContainer().getName(),
            sendingPort.getVariable().getName(),
            connection.getDstFederate().id,
            connection.getDstFederate().networkMessageActions.size(),
            getNetworkDelayLiteral(connection.getDefinition().getDelay())
        );
    }

    private String getNetworkDelayLiteral(Expression e) {
        var cLiteral = CExtensionUtils.getNetworkDelayLiteral(e);
        return cLiteral.equals("NEVER") ? "0" : cLiteral;
    }

    @Override
    public String generateNetworkInputControlReactionBody(int receivingPortID, TimeValue maxSTP, CoordinationType coordination) {
        return "// TODO(hokeun): Figure out what to do for generateNetworkInputControlReactionBody";
    }

    @Override
    public String generateNetworkOutputControlReactionBody(VarRef srcOutputPort, FedConnectionInstance connection) {
        return "// TODO(hokeun): Figure out what to do for generateNetworkOutputControlReactionBody";
    }

    @Override
    public String getNetworkBufferType() {
        return "";
    }

    /**
     * Add necessary preamble to the source to set up federated execution.
     *
     * @return
     */
    @Override
    public String generatePreamble(FederateInstance federate, FedFileConfig fileConfig,
                                   LinkedHashMap<String, Object> federationRTIProperties,
                                   ErrorReporter errorReporter) {
        var minOutputDelay = getMinOutputDelay(federate, fileConfig, errorReporter);
        return
        """
            preamble {=
                const defaultFederateConfig: __FederateConfig = {
                    dependsOn: [%s],
                    executionTimeout: undefined,
                    fast: false,
                    federateID: %d,
                    federationID: "Unidentified Federation",
                    keepAlive: false,
                    minOutputDelay: %s,
                    networkMessageActions: [%s],
                    rtiHost: "%s",
                    rtiPort: %d,
                    sendsTo: [%s]
                }
            =}""".formatted(
            federate.dependsOn.keySet().stream()
                              .map(e->String.valueOf(e.id))
                              .collect(Collectors.joining(",")),
            federate.id,
            minOutputDelay == null ? "undefined"
                                   : "%s".formatted(TSExtensionsKt.toTsTime(minOutputDelay)),
            federate.networkMessageActions
                .stream()
                .map(Variable::getName)
                .collect(Collectors.joining(",", "\"", "\"")),
            federationRTIProperties.get("host"),
            federationRTIProperties.get("port"),
            federate.sendsTo.keySet().stream()
                            .map(e->String.valueOf(e.id))
                            .collect(Collectors.joining(","))
        );
    }

    private TimeValue getMinOutputDelay(FederateInstance federate, FedFileConfig fileConfig, ErrorReporter errorReporter) {
        if (federate.targetConfig.coordination.equals(CoordinationType.CENTRALIZED)) {
            // If this program uses centralized coordination then check
            // for outputs that depend on physical actions so that null messages can be
            // sent to the RTI.
            var federateClass = ASTUtils.toDefinition(federate.instantiation.getReactorClass());
            var main = new ReactorInstance(FedASTUtils.findFederatedReactor(federate.instantiation.eResource()), errorReporter, 1);
            var instance = new ReactorInstance(federateClass, main, errorReporter);
            var outputDelayMap = federate
                .findOutputsConnectedToPhysicalActions(instance);
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
                if (federate.targetConfig.coordinationOptions.advance_message_interval
                    == null) {
                    errorReporter.reportWarning(outputFound, String.join("\n",
                                                                         "Found a path from a physical action to output for reactor "
                                                                             + addDoubleQuotes(instance.getName())
                                                                             + ". ",
                                                                         "The amount of delay is "
                                                                             + minOutputDelay
                                                                             + ".",
                                                                         "With centralized coordination, this can result in a large number of messages to the RTI.",
                                                                         "Consider refactoring the code so that the output does not depend on the physical action,",
                                                                         "or consider using decentralized coordination. To silence this warning, set the target",
                                                                         "parameter coordination-options with a value like {advance-message-interval: 10 msec}"
                    ));
                }
                return minOutputDelay;
            }
        }
        return null;
    }
}
