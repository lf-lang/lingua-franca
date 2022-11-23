package org.lflang.generator.ts

import org.lflang.*
import org.lflang.federated.FederateInstance
import org.lflang.generator.PrependOperator
import org.lflang.generator.ReactorInstance
import org.lflang.lf.*
import java.util.*

/**
 * Reactor generator for TypeScript target.
 *
 *  @author{Matt Weber <matt.weber@berkeley.edu>}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author {Christian Menard <christian.menard@tu-dresden.de>}
 *  @author {Hokeun Kim <hokeunkim@berkeley.edu>}
 */
class TSReactorGenerator(
    private val tsGenerator: TSGenerator,
    private val errorReporter: ErrorReporter,
    private val targetConfig: TargetConfig
) {
    // Initializer functions
    fun getTargetInitializerHelper(param: Parameter,
                                   list: List<String>): String {
        return if (list.isEmpty()) {
            errorReporter.reportError(param, "Parameters must have a default value!")
        } else if (list.size == 1) {
            list[0]
        } else {
            list.joinToString(", ", "[", "]")
        }
    }

    /** Generate the main app instance. This function is only used once
     *  because all other reactors are instantiated as properties of the
     *  main one.
     *  @param instance A reactor instance.
     */
    private fun generateReactorInstance(
        defn: Instantiation,
        mainParameters: Set<Parameter>
    ): String {

        val fullName = defn.name

        // Iterate through parameters in the order they appear in the
        // main reactor class. If the parameter is typed such that it can
        // be a custom command line argument, use the parameter's command line
        // assignment variable ("__CL" + the parameter's name). That variable will
        // be undefined if the command line argument wasn't specified. Otherwise
        // use undefined in the constructor.
        val mainReactorParams = StringJoiner(", ")
        for (parameter in defn.reactorClass.toDefinition().parameters) {

            if (mainParameters.contains(parameter)) {
                mainReactorParams.add("__CL" + parameter.name)
            } else {
                mainReactorParams.add("undefined")
            }
        }

        return with(PrependOperator) {
            """
            |// ************* Instance $fullName of class ${defn.reactorClass.name}
            |let __app;
            |if (!__noStart) {
            |    __app = new $fullName(__timeout, __keepAlive, __fast, __federationID, $mainReactorParams);
            |}
            """
        }.trimMargin()
    }

    /** Generate code to call the _start function on the main App
     *  instance to start the runtime
     *  @param instance A reactor instance.
     */
    private fun generateRuntimeStart(federate: FederateInstance,
                                     main: ReactorInstance?,
                                     defn: Instantiation): String {
        var minOutputDelay = TimeValue.MAX_VALUE;
        if (tsGenerator.isFederatedAndCentralized && main != null) {
            // Check for outputs that depend on physical actions.
            for (reactorInstance in main.children) {
                if (federate.contains(reactorInstance)) {
                    val outputDelayMap = federate.findOutputsConnectedToPhysicalActions(reactorInstance)
                    for (outputDelay in outputDelayMap.values) {
                        if (outputDelay.isEarlierThan(minOutputDelay)) {
                            minOutputDelay = outputDelay
                        }
                    }
                }
            }
        }

        if (minOutputDelay != TimeValue.MAX_VALUE && targetConfig.coordinationOptions.advance_message_interval == null) {
            // There is a path from a physical action to output for reactor but advance message interval is not set.
            // Report a warning.
            errorReporter.reportWarning(
                """
                    Found a path from a physical action to output for reactor ${defn.name}.
                    The amount of delay is $minOutputDelay.
                    With centralized coordination, this can result in a large number of messages to the RTI.
                    Consider refactoring the code so that the output does not depend on the physical action,
                    or consider using decentralized coordination. To silence this warning, set the target
                    parameter coordination-options with a value like {advance-message-interval: 10 msec}
                """.trimIndent()
            )

        }

        return with(PrependOperator) {
                """
            |// ************* Starting Runtime for ${defn.name} + of class ${defn.reactorClass.name}.
            |if (!__noStart && __app) {
            |    ${if (minOutputDelay == TimeValue.MAX_VALUE) "" else "__app.setMinDelayFromPhysicalActionToFederateOutput(${TSGenerator.timeInTargetLanguage(minOutputDelay)})"}
            |    __app._start();
            |}
            |
            """
            }.trimMargin()
    }

    private fun generateReactorPreambles(preambles: List<Preamble>): String {
        val preambleCodes = LinkedList<String>()

        for (preamble in preambles) {
            preambleCodes.add(with(PrependOperator) {
                """
                |// *********** From the preamble, verbatim:
                |${preamble.code.toText()}
                |
                |// *********** End of preamble."""}.trimMargin())
        }
        return preambleCodes.joinToString("\n")
    }

    fun generateReactor(reactor: Reactor, federate: FederateInstance): String {
        var reactorName = reactor.name
        if (!reactor.typeParms.isEmpty()) {
            reactorName +=
                reactor.typeParms.joinToString(", ", "<", ">") { it.toText() }
        }

        // NOTE: type parameters that are referenced in ports or actions must extend
        // Present in order for the program to type check.
        val classDefinition: String = if (reactor.isMain()) {
            "class $reactorName extends __App {"
        } else if (reactor.isFederated) {
            "class $reactorName extends __FederatedApp {"
        } else {
            "export class $reactorName extends __Reactor {"
        }

        val instanceGenerator = TSInstanceGenerator(errorReporter, reactor, federate)
        val timerGenerator = TSTimerGenerator(reactor.timers)
        val parameterGenerator = TSParameterGenerator(reactor.parameters)
        val stateGenerator = TSStateGenerator(reactor.stateVars)
        val actionGenerator = TSActionGenerator(reactor.actions, federate)
        val portGenerator = TSPortGenerator(reactor.inputs, reactor.outputs)

        val constructorGenerator = TSConstructorGenerator(tsGenerator, errorReporter, reactor, federate, targetConfig)
        return with(PrependOperator) {
            """
                |// =============== START reactor class ${reactor.name}
                |${generateReactorPreambles(reactor.preambles)}
                |
                |$classDefinition
            ${" |    "..instanceGenerator.generateClassProperties()}
            ${" |    "..timerGenerator.generateClassProperties()}
            ${" |    "..parameterGenerator.generateClassProperties()}
            ${" |    "..stateGenerator.generateClassProperties()}
            ${" |    "..actionGenerator.generateClassProperties()}
            ${" |    "..portGenerator.generateClassProperties()}
            ${" |    "..constructorGenerator.generateConstructor(instanceGenerator, timerGenerator, parameterGenerator,
                stateGenerator, actionGenerator, portGenerator)}
                |}
                |// =============== END reactor class ${reactor.name}
                |
            """.trimMargin()
        }
    }

    fun generateReactorInstanceAndStart(
        federate: FederateInstance,
        main: ReactorInstance?,
        mainDef: Instantiation,
        mainParameters: Set<Parameter>
    ): String {
        return with(PrependOperator) {
            """
            |${generateReactorInstance(mainDef, mainParameters)}
            |${generateRuntimeStart(federate, main, mainDef)}
            |
            """
        }.trimMargin()
    }
}
