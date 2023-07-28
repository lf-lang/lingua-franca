package org.lflang.generator.ts

import org.lflang.MessageReporter
import org.lflang.TargetConfig
import org.lflang.generator.PrependOperator
import org.lflang.generator.getTargetInitializer
import org.lflang.joinWithLn
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
class TSConstructorGenerator(
    private val messageReporter: MessageReporter,
    private val reactor: Reactor
) {

    private fun initializeParameter(p: Parameter): String =
        "${p.name}: ${TSTypes.getInstance().getTargetType(p)} = ${TSTypes.getInstance().getTargetInitializer(p)}"

    private fun generateConstructorArguments(reactor: Reactor): String {
        val arguments = StringJoiner(", \n")
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

        return arguments.toString()
    }

    private fun generateSuperConstructorCall(reactor: Reactor, isFederate: Boolean): String =
        if (reactor.isMain) {
            if (isFederate) {
                """
                    |        var federateConfig = defaultFederateConfig;
                    |        if (__timeout !== undefined) {
                    |            federateConfig.executionTimeout = __timeout;
                    |        }
                    |        federateConfig.federationID = __federationID;
                    |        federateConfig.fast = __fast;
                    |        super(federateConfig, success, fail);
                    """.trimMargin()
            } else {
                "super(timeout, keepAlive, fast, success, fail);"
            }
        } else {
            "super(parent);"
        }

    // Generate code for setting target configurations.
    private fun generateTargetConfigurations(targetConfig: TargetConfig): String {
        val interval = targetConfig.coordinationOptions.advance_message_interval
        return if ((reactor.isMain) && interval != null) {
            "this.setAdvanceMessageInterval(${interval.toTsTime()})"
        } else ""
    }

    fun generateConstructor(
        targetConfig: TargetConfig,
        instances: TSInstanceGenerator,
        timers: TSTimerGenerator,
        parameters: TSParameterGenerator,
        states: TSStateGenerator,
        actions: TSActionGenerator,
        ports: TSPortGenerator,
        isFederate: Boolean,
        isNetworkReceiver: Boolean
    ): String {
        val connections = TSConnectionGenerator(reactor.connections, messageReporter)
        val reactions = TSReactionGenerator(messageReporter, reactor)

        return with(PrependOperator) {
            """
                |constructor (
            ${" |    "..generateConstructorArguments(reactor)}
                |) {
            ${" |    "..generateSuperConstructorCall(reactor, isFederate)}
            ${" |    "..generateTargetConfigurations(targetConfig)}
            ${" |    "..instances.generateInstantiations()}
            ${" |    "..timers.generateInstantiations()}
            ${" |    "..parameters.generateInstantiations()}
            ${" |    "..states.generateInstantiations()}
            ${" |    "..actions.generateInstantiations(isNetworkReceiver)}
            ${" |    "..ports.generateInstantiations()}
            ${" |    "..connections.generateInstantiations()}
            ${" |    "..reactions.generateAllReactions()}
                |}
            """.trimMargin()
        }
    }
}
