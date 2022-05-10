package org.lflang.generator.ts

import org.lflang.ErrorReporter
import org.lflang.federated.FederateInstance
import org.lflang.generator.PrependOperator
import org.lflang.lf.Action
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
    private val reactor : Reactor,
    private val federate: FederateInstance
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

    private fun federationRTIProperties(): LinkedHashMap<String, Any> {
        return tsGenerator.federationRTIPropertiesW()
    }

    private fun generateSuperConstructorCall(reactor: Reactor, federate: FederateInstance): String {
        if (reactor.isMain) {
            return "super(timeout, keepAlive, fast, success, fail);"
        } else if (reactor.isFederated) {
            var port = federationRTIProperties()["port"]
            // Default of 0 is an indicator to use the default port, 15045.
            if (port == 0) {
                port = 15045
            }
            return """
            super(federationID, ${federate.id}, ${port},
                "${federationRTIProperties()["host"]}",
                timeout, keepAlive, fast, success, fail);
            """
        } else {
            return "super(parent);"
        }
    }

    // If the app is federated, register its
    // networkMessageActions with the RTIClient
    private fun generateFederatePortActionRegistrations(networkMessageActions: List<Action>): String {
        var fedPortID = 0;
        val connectionInstantiations = LinkedList<String>()
        for (nAction in networkMessageActions) {
            val registration = """
                this.registerFederatePortAction(${fedPortID}, this.${nAction.name});
                """
            connectionInstantiations.add(registration)
            fedPortID++
        }
        return connectionInstantiations.joinToString("\n")
    }

    // Generate code for registering Fed IDs that are connected to
    // this federate via ports in the TypeScript's FederatedApp.
    // These Fed IDs are used to let the RTI know about the connections
    // between federates during the initialization with the RTI.
    fun generateFederateConfigurations(): String {
        val federateConfigurations = LinkedList<String>()
        if (reactor.isFederated) {
            for ((key, _) in federate.dependsOn) {
                // FIXME: Get delay properly considering the unit instead of hardcoded BigInt(0).
                federateConfigurations.add("this.addUpstreamFederate(${key.id}, BigInt(0));")
            }
            for ((key, _) in federate.sendsTo) {
                federateConfigurations.add("this.addDownstreamFederate(${key.id});")
            }
        }
        return federateConfigurations.joinToString("\n")
    }

    fun generateConstructor(
        instances: TSInstanceGenerator,
        timers: TSTimerGenerator,
        parameters: TSParameterGenerator,
        states: TSStateGenerator,
        actions: TSActionGenerator,
        ports: TSPortGenerator
    ): String {
        val connections = TSConnectionGenerator(reactor.connections, errorReporter)
        val reactions = TSReactionGenerator(tsGenerator, errorReporter, reactor, federate)

        return with(PrependOperator) {
            """
                |constructor (
            ${" |    "..generateConstructorArguments(reactor)}
                |) {
            ${" |    "..generateSuperConstructorCall(reactor, federate)}
            ${" |    "..generateFederateConfigurations()}
            ${" |    "..instances.generateInstantiations()}
            ${" |    "..timers.generateInstantiations()}
            ${" |    "..parameters.generateInstantiations()}
            ${" |    "..states.generateInstantiations()}
            ${" |    "..actions.generateInstantiations(federate.networkMessageActions)}
            ${" |    "..ports.generateInstantiations()}
            ${" |    "..connections.generateInstantiations()}
            ${" |    "..if (reactor.isFederated) generateFederatePortActionRegistrations(federate.networkMessageActions) else ""}
            ${" |    "..reactions.generateAllReactions()}
                |}
            """.trimMargin()
        }
    }
}