package org.lflang.generator.ts

import org.lflang.ErrorReporter
import org.lflang.TargetConfig
import org.lflang.generator.PrependOperator
import org.lflang.lf.Parameter
import org.lflang.lf.Reactor
import java.util.*

/**
 * Generator for code in the constructor of reactor class in TypeScript target.
 * Specifically, this generator generates the code for constructor argument,
 * call to the super class constructor. This generator uses other code generators
 * for child reactors, timers, parameters, state variables, actions, ports, connections,
 * and code to register reactions. This generator also generates federate port action
 * registrations.
 */
class TSConstructorGenerator (
    private val tsGenerator: TSGenerator,
    private val errorReporter: ErrorReporter,
    private val reactor : Reactor
) {
    private fun getInitializerList(param: Parameter): List<String> =
        tsGenerator.getInitializerListW(param)

    // Initializer functions
    private fun getTargetInitializerHelper(param: Parameter,
                                   list: List<String>): String {
        return if (list.size == 0) {
            errorReporter.reportError(param, "Parameters must have a default value!")
        } else if (list.size == 1) {
            list[0]
        } else {
            list.joinToString(", ", "[", "]")
        }
    }
    private fun getTargetInitializer(param: Parameter): String {
        return getTargetInitializerHelper(param, getInitializerList(param))
    }
    private fun initializeParameter(p: Parameter): String {
        return """${p.name}: ${p.getTargetType()} = ${getTargetInitializer(p)}"""
    }

    private fun generateConstructorArguments(reactor: Reactor): String {
        val arguments = LinkedList<String>()
        if (reactor.isMain || reactor.isFederated) {
            arguments.add("timeout: TimeValue | undefined = undefined")
            arguments.add("keepAlive: boolean = false")
            arguments.add("fast: boolean = false")
            arguments.add("federationID: string = 'Unidentified Federation'")
        } else {
            arguments.add("parent: __Reactor")
        }

        // For TS, parameters are arguments of the class constructor.
        for (parameter in reactor.parameters) {
            arguments.add(initializeParameter(parameter))
        }

        if (reactor.isMain || reactor.isFederated) {
            arguments.add("success?: () => void")
            arguments.add("fail?: () => void")
        }

        return arguments.joinToString(", \n")
    }

    private fun generateSuperConstructorCall(reactor: Reactor, federateConfig: TSFederateConfig?): String {
        if (reactor.isMain) {
            if (federateConfig != null) {
                return """
                    super(federationID, ${federateConfig.getFederateId()}, ${federateConfig.getRtiPort()},
                        "${federateConfig.getRtiHost()}",
                        timeout, keepAlive, fast, success, fail);
                    """

            } else {
                return "super(timeout, keepAlive, fast, success, fail);"
            }
        }
        else {
            return "super(parent);"
        }
    }

    // If the app is federated, register its
    // networkMessageActions with the RTIClient
    private fun generateFederatePortActionRegistrations(federateConfig: TSFederateConfig): String {
        val connectionInstantiations = LinkedList<String>()
        var fedPortID = 0
        for (actionName in federateConfig.getNetworkMessageActions()) {
            val registration = """
                this.registerFederatePortAction(${fedPortID}, this.${actionName});
                """
            connectionInstantiations.add(registration)
            fedPortID++
        }
        return connectionInstantiations.joinToString("\n")
    }

    /**
     * Generate code for setting target configurations.
     */
    private fun generateTargetConfigurations(targetConfig: TargetConfig): String {
        val targetConfigurations = LinkedList<String>()
        if ((reactor.isMain) &&
            targetConfig.coordinationOptions.advance_message_interval != null) {
            targetConfigurations.add(
                "this.setAdvanceMessageInterval(${timeInTargetLanguage(targetConfig.coordinationOptions.advance_message_interval)})")
        }
        return targetConfigurations.joinToString("\n")
        return "";
    }

    // Generate code for registering Fed IDs that are connected to
    // this federate via ports in the TypeScript's FederatedApp.
    // These Fed IDs are used to let the RTI know about the connections
    // between federates during the initialization with the RTI.
    fun generateFederateConfigurations(federateConfig: TSFederateConfig): String {
        val federateConfigurations = LinkedList<String>()
        for (id in federateConfig.getDependOnFedIds()) {
            // FIXME: Get delay properly considering the unit instead of hardcoded BigInt(0).
            federateConfigurations.add("this.addUpstreamFederate($id, BigInt(0));")
        }
        for (id in federateConfig.getSendsToFedIds()) {
            federateConfigurations.add("this.addDownstreamFederate($id);")
        }
        return federateConfigurations.joinToString("\n")
    }

    // FIXME: port-absent
    // Generate code for registering Fed IDs that are connected to
    // this federate via ports in the TypeScript's FederatedApp.
    // These Fed IDs are used to let the RTI know about the connections
    // between federates during the initialization with the RTI.
    fun generateNetworkControlActionRegistrations(federateConfig: TSFederateConfig): String {
        val connectionInstantiations= LinkedList<String>()
        for (id in federateConfig.getnetworkInputControlReactionsTriggers()) {
            connectionInstantiations.add("this.registerInputControlReactionTrigger(this.$id);")
        }
        if (federateConfig.getSendsToFedIds().size != 0) {
            connectionInstantiations.add("this.registerOutputControlReactionTrigger(this.outputControlReactionTrigger);")
        }
        return connectionInstantiations.joinToString("\n")
    }

    fun generateConstructor(
        targetConfig: TargetConfig,
        instances: TSInstanceGenerator,
        timers: TSTimerGenerator,
        parameters: TSParameterGenerator,
        states: TSStateGenerator,
        actions: TSActionGenerator,
        ports: TSPortGenerator,
        federateConfig: TSFederateConfig?
    ): String {
        val connections = TSConnectionGenerator(reactor.connections, errorReporter)
        val reactions = TSReactionGenerator(tsGenerator, errorReporter, reactor)

        return with(PrependOperator) {
            """
                |constructor (
            ${" |    "..generateConstructorArguments(reactor)}
                |) {
            ${" |    "..generateSuperConstructorCall(reactor, federateConfig)}
            ${" |    "..generateTargetConfigurations(targetConfig)}
            ${" |    "..if (reactor.isMain && federateConfig != null) generateFederateConfigurations(federateConfig) else ""}
            ${" |    "..instances.generateInstantiations()}
            ${" |    "..timers.generateInstantiations()}
            ${" |    "..parameters.generateInstantiations()}
            ${" |    "..states.generateInstantiations()}
            ${" |    "..actions.generateInstantiations()}
            ${" |    "..ports.generateInstantiations()}
            ${" |    "..connections.generateInstantiations()}
            ${" |    "..if (reactor.isMain && federateConfig != null) generateFederatePortActionRegistrations(federateConfig) else ""}
            ${" |    "..if (reactor.isMain && federateConfig != null) generateNetworkControlActionRegistrations(federateConfig) else ""}
            ${" |    "..reactions.generateAllReactions()}
                |}
            """.trimMargin()
        }
    }
}
