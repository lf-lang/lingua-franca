package org.lflang.generator.ts

import org.lflang.ASTUtils.toText
import org.lflang.ErrorReporter
import org.lflang.generator.FederateInstance
import org.lflang.generator.cpp.CppParameterGenerator.Companion.targetType
import org.lflang.inferredType
import org.lflang.isInitialized
import org.lflang.lf.*
import org.lflang.lf.Timer
import org.lflang.toText
import java.lang.StringBuilder
import java.util.*
import kotlin.collections.LinkedHashMap

class TsReactorGenerator(
    private val tsGenerator: TsGenerator,
    private val errorReporter: ErrorReporter
) {
    private var code = StringBuilder()

    /**
     * Map from builder to its current indentation.
     */
    var indentation = LinkedHashMap<StringBuilder, String>()

    private fun indent(builder: StringBuilder) {
        tsGenerator.indentw(builder)
    }
    private fun unindent(builder: StringBuilder) {
        tsGenerator.unindentw(builder)
    }
    private fun indent() {
        indent(code)
    }
    private fun unindent() {
        unindent(code)
    }

    private fun pr(builder: StringBuilder, text: Any) {
        tsGenerator.prw(builder, text)
    }
    private fun pr(text: Any) {
        tsGenerator.prw(code, text)
    }

    private fun federationRTIProperties(): LinkedHashMap<String, Any> {
        return tsGenerator.federationRTIPropertiesW()
    }
    fun getTargetValue(v: Value): String {
        return tsGenerator.getTargetValueW(v)
    }

    fun getTargetType(p: Parameter): String {
        return tsGenerator.getTargetTypeW(p)
    }
    fun getTargetType(state: StateVar): String {
        return tsGenerator.getTargetTypeW(state)
    }
    fun getTargetType(a: Action): String {
        return tsGenerator.getTargetTypeW(a)
    }
    fun getTargetType(p: Port): String {
        return tsGenerator.getTargetTypeW(p)
    }
    fun getTargetType(t: Type): String {
        return tsGenerator.getTargetTypeW(t)
    }
    fun generateVarRef(reference: VarRef): String {
        return tsGenerator.generateVarRef(reference)
    }

    private fun getInitializerList(state: StateVar): List<String> {
        return tsGenerator.getInitializerListW(state)
    }
    fun getInitializerList(param: Parameter): List<String> {
        return tsGenerator.getInitializerListW(param)
    }
    private fun getTargetInitializer(state: StateVar): String {
        return getInitializerList(state).joinToString(",")
    }

    private fun getTargetInitializerHelper(param: Parameter,
                                           list: List<String>): String {
        return if (list.size == 0) {
            errorReporter.reportError(param, "Parameters must have a default value!")
        } else if (list.size == 1) {
            list.get(0)
        } else {
            list.joinToString(",")
        }
    }
    private fun getTargetInitializer(param: Parameter): String {
        return getTargetInitializerHelper(param, getInitializerList(param))
    }
    private fun initializeParameter(p: Parameter): String {
        return """${p.name}: ${p.targetType} = ${getTargetInitializer(p)}"""
    }

    /**
     * Return a TS type for the specified action.
     * If the type has not been specified, return
     * "Present" which is the base type for Actions.
     * @param action The action
     * @return The TS type.
     */
    private fun getActionType(action: Action): String {
        if (action.type !== null) {
            return getTargetType(action.type)
        } else {
            return "Present"
        }
    }

    /**
     * Return a TS type for the specified port.
     * If the type has not been specified, return
     * "Present" which is the base type for ports.
     * @param port The port
     * @return The TS type.
     */
    private fun getPortType(port: Port): String {
        if (port.type !== null) {
            return getTargetType(port.type)
        } else {
            return "Present"
        }
    }

    private fun generateArg(v: VarRef): String {
        return if (v.container !== null) {
            """__${v.container.name}_${v.variable.name}"""
        } else {
            """__${v.variable.name}"""
        }
    }

    fun generateReactor(reactor: Reactor, federate: FederateInstance): String {

        // TODO(hokeun): Append parameters after the name.
        val reactorName = reactor.name
        // NOTE: type parameters that are referenced in ports or actions must extend
        // Present in order for the program to type check.
        if (reactor.isMain()) {
            pr("class " + reactorName + " extends __App {")
        } else if (reactor.isFederated()) {
            pr("class " + reactorName + " extends __FederatedApp {")
        } else {
            pr("export class " + reactorName + " extends __Reactor {")
        }

        indent()

        var arguments = LinkedList<String>()
        if (reactor.isMain() || reactor.isFederated()) {
            arguments.add("timeout: TimeValue | undefined = undefined")
            arguments.add("keepAlive: boolean = false")
            arguments.add("fast: boolean = false")
        } else {
            arguments.add("parent: __Reactor")
        }

        // For TS, parameters are arguments of the class constructor.
        for (parameter in reactor.parameters) {
            arguments.add(initializeParameter(parameter))
        }

        var reactorConstructor = StringBuilder()
        if (reactor.isMain() || reactor.isFederated()) {
            arguments.add("success?: () => void")
            arguments.add("fail?: () => void")
            pr(reactorConstructor, "constructor (")
            indent(reactorConstructor)
            pr(reactorConstructor, arguments.joinToString(", \n"))
            unindent(reactorConstructor)
            pr(reactorConstructor, ") {")
            indent(reactorConstructor)
        } else {
            pr(reactorConstructor, "constructor (")
            indent(reactorConstructor)
            pr(reactorConstructor, arguments.joinToString(", \n"))
            unindent(reactorConstructor)
            pr(reactorConstructor, ") {")
            indent(reactorConstructor)
        }

        var superCall: String
        if (reactor.isMain()) {
            superCall = "super(timeout, keepAlive, fast, success, fail);"
        } else if (reactor.isFederated()) {
            var port = federationRTIProperties()["port"]
            // Default of 0 is an indicator to use the default port, 15045.
            if (port === 0) {
                port = 15045
            }
            superCall = """
            super(${federate.id}, ${port},
                "${federationRTIProperties()["host"]}",
                timeout, keepAlive, fast, success, fail);
            """
        } else {
            superCall = "super(parent);"
        }
        pr(reactorConstructor, superCall)

        // Next handle child reactors instantiations.
        // If the app isn't federated, instantiate all
        // the child reactors. If the app is federated

        var childReactors: List<Instantiation>
        if (!reactor.isFederated()) {
            childReactors = reactor.instantiations
        } else {
            childReactors = LinkedList<Instantiation>()
            childReactors.add(federate.instantiation)
        }

        for (childReactor in childReactors) {
            pr(childReactor.getName() + ": " + childReactor.reactorClass.name +
                    childReactor.typeParms.joinToString(","))

            var childReactorArguments = StringJoiner(", ");
            childReactorArguments.add("this")
/*
            // TODO(hokeun): Uncomment this part and fix errors.
            // Iterate through parameters in the order they appear in the
            // reactor class, find the matching parameter assignments in
            // the reactor instance, and write the corresponding parameter
            // value as an argument for the TypeScript constructor
            for (parameter in childReactor.reactorClass.toDefinition.parameters) {
                childReactorArguments.add(parameter.getTargetInitializer(childReactor))
            }
*/
            pr(reactorConstructor, "this." + childReactor.getName()
                    + " = new " + childReactor.reactorClass.name +
                    "(" + childReactorArguments + ")" )
        }

        // Next handle timers.
        for (timer in reactor.timers) {
            var timerPeriod: String
            if (timer.period === null) {
                timerPeriod = "0"
            } else {
                timerPeriod = getTargetValue(timer.period)
            }

            var timerOffset: String
            if (timer.offset === null) {
                timerOffset = "0"
            } else {

                timerOffset = getTargetValue(timer.offset)

            }

            pr(timer.getName() + ": __Timer;")
            pr(reactorConstructor, "this." + timer.getName()
                    + " = new __Timer(this, " + timerOffset + ", "+ timerPeriod + ");")

        }

        // Create properties for parameters
        for (param in reactor.parameters) {
            pr(param.name + ": __Parameter<" + getTargetType(param) + ">;")
            pr(reactorConstructor, "this." + param.name +
                    " = new __Parameter(" + param.name + ");" )
        }

        // Next handle states.
        for (stateVar in reactor.stateVars) {
            if (stateVar.isInitialized) {
                pr(reactorConstructor, "this." + stateVar.name + " = " +
                        "new __State(" + getTargetInitializer(stateVar) + ");");
            } else {
                pr(reactorConstructor, "this." + stateVar.name + " = " +
                        "new __State(undefined);");
            }
        }

        for (stateVar in reactor.stateVars) {
            pr(stateVar.name + ": " + "__State<" + getTargetType(stateVar) + ">;");
        }

        // Next handle actions.
        for (action in reactor.actions) {
            // Shutdown actions are handled internally by the
            // TypeScript reactor framework. There would be a
            // duplicate action if we included the one generated
            // by LF.
            if (action.name != "shutdown") {
                pr(action.name + ": __Action<" + getActionType(action) + ">;")

                var actionArgs = "this, __Origin." + action.origin
                if (action.minDelay !== null) {
                    // Actions in the TypeScript target are constructed
                    // with an optional minDelay argument which defaults to 0.
                    if (action.minDelay.parameter !== null) {
                        actionArgs+= ", " + action.minDelay.parameter.name
                    } else {
                        actionArgs+= ", " + getTargetValue(action.minDelay)
                    }
                }
                pr(reactorConstructor, "this." +
                        action.name + " = new __Action<" + getActionType(action) +
                        ">(" + actionArgs  + ");")
            }
        }

        // Next handle inputs.
        for (input in reactor.inputs) {
            pr(input.name + ": " + "__InPort<" + getPortType(input) + ">;")
            pr(reactorConstructor, "this." + input.name + " = new __InPort<"
                    + getPortType(input) + ">(this);")
        }

        // Next handle outputs.
        for (output in reactor.outputs) {
            pr(output.name + ": " + "__OutPort<" + getPortType(output) + ">;")
            pr(reactorConstructor, "this." + output.name + " = new __OutPort<"
                    + getPortType(output) + ">(this);")
        }

        // Next handle connections
        for (connection in reactor.connections) {
            var leftPortName = ""
            // FIXME: Add support for multiports.
            if (connection.leftPorts.size > 1) {
                errorReporter.reportError(connection, "Multiports are not yet supported in the TypeScript target.")
            } else {
                if (connection.leftPorts.get(0).container !== null) {
                    leftPortName += connection.leftPorts.get(0).container.name + "."
                }
                leftPortName += connection.leftPorts.get(0).variable.name
            }
            var rightPortName = ""
            if (connection.leftPorts.size > 1) {
                errorReporter.reportError(connection, "Multiports are not yet supported in the TypeScript target.")
            } else {
                if (connection.rightPorts.get(0).container !== null) {
                    rightPortName += connection.rightPorts.get(0).container.name + "."
                }
                rightPortName += connection.rightPorts.get(0).variable.name
            }
            if (leftPortName != "" && rightPortName != "") {
                pr(reactorConstructor, "this._connect(this." + leftPortName + ", this." + rightPortName + ");")
            }
        }

        // If the app is federated, register its
        // networkMessageActions with the RTIClient
        if (reactor.isFederated()) {
            // The ID of the receiving port is simply
            // the position of the action in the networkMessageActions list.
            var fedPortID = 0;
            for (nAction in federate.networkMessageActions) {
                var registration = """
                this.registerFederatePortAction(${fedPortID}, this.${nAction.name});
                """
                pr(reactorConstructor, registration)
                fedPortID++
            }
        }

        // Next handle reaction instances.
        // If the app is federated, only generate
        // reactions that are contained by that federate
        var generatedReactions: List<Reaction>
        if (reactor.isFederated()) {
            generatedReactions = LinkedList<Reaction>()
            for (reaction in reactor.reactions) {
                if (federate.containsReaction(reactor, reaction)) {
                    generatedReactions.add(reaction)
                }
            }
        } else {
            generatedReactions = reactor.reactions
        }

        ///////////////////// Reaction generation begins /////////////////////
        // TODO(hokeun): Consider separating this out as a new class.
        for (reaction in generatedReactions) {

            // Determine signature of the react function
            var reactSignature = StringJoiner(", ")
            reactSignature.add("this")

            // The prologue to the react function writes state
            // and parameters to local variables of the same name
            var reactPrologue = StringBuilder()
            pr(reactPrologue, "const util = this.util;")

            // The epilogue to the react function writes local
            // state variables back to the state
            var reactEpilogue = StringBuilder()

            // Assemble react function arguments from sources and effects
            // Arguments are either elements of this reactor, or an object
            // representing a contained reactor with properties corresponding
            // to listed sources and effects.

            // If a source or effect is an element of this reactor, add it
            // directly to the reactFunctArgs string. If it isn't, write it
            // into the containerToArgs map, and add it to the string later.
            var reactFunctArgs = StringJoiner(", ")
            // Combine triggers and sources into a set
            // so we can iterate over their union
            var triggersUnionSources = HashSet<VarRef>()
            for (trigger in reaction.triggers) {
                if (!(trigger.isStartup || trigger.isShutdown)) {
                    triggersUnionSources.add(trigger as VarRef)
                }
            }
            for (source in reaction.sources) {
                triggersUnionSources.add(source)
            }

            // Create a set of effect names so actions that appear
            // as both triggers/sources and effects can be
            // identified and added to the reaction arguments once.
            // We can't create a set of VarRefs because
            // an effect and a trigger/source with the same name are
            // unequal.
            // The key of the pair is the effect's container's name,
            // The effect of the pair is the effect's name
            var effectSet = HashSet<Pair<String, String>>()

            for (effect in reaction.effects) {
                var key = ""; // The container, defaults to an empty string
                var value = effect.variable.name; // The name of the effect
                if (effect.container !== null) {
                    key = effect.container.name
                }
                effectSet.add(Pair(key, value))
            }

            // Add triggers and sources to the react function
            var containerToArgs = HashMap<Instantiation, HashSet<Variable>>();
            for (trigOrSource in triggersUnionSources) {
                // Actions that are both read and scheduled should only
                // appear once as a schedulable effect

                var trigOrSourceKey = "" // The default for no container
                var trigOrSourceValue = trigOrSource.variable.name
                if (trigOrSource.container !== null) {
                    trigOrSourceKey = trigOrSource.container.name
                }
                var trigOrSourcePair = Pair(trigOrSourceKey, trigOrSourceValue)

                if (!effectSet.contains(trigOrSourcePair)) {
                    var reactSignatureElementType = "";

                    if (trigOrSource.variable is Timer) {
                        reactSignatureElementType = "__Tag"
                    } else if (trigOrSource.variable is Action) {
                        reactSignatureElementType = getActionType(trigOrSource.variable as Action)
                    } else if (trigOrSource.variable is Port) {
                        reactSignatureElementType = getPortType(trigOrSource.variable as Port)
                    }

                    reactSignature.add("""${generateArg(trigOrSource)}: Read<${reactSignatureElementType}>""")
                    reactFunctArgs.add("this." + generateVarRef(trigOrSource))
                    if (trigOrSource.container === null) {
                        pr(reactPrologue, """let ${trigOrSource.variable.name} = ${generateArg(trigOrSource)}.get();""")
                    } else {
                        var args = containerToArgs.get(trigOrSource.container)
                        if (args === null) {
                            // Create the HashSet for the container
                            // and handle it later.
                            args = HashSet<Variable>();
                            containerToArgs.put(trigOrSource.container, args)
                        }
                        args.add(trigOrSource.variable)
                    }
                }
            }
            var schedActionSet = HashSet<Action>();
            for (effect in reaction.effects) {
                var functArg = ""
                var reactSignatureElement = "" + generateArg(effect)
                if (effect.variable is Timer) {
                    errorReporter.reportError("A timer cannot be an effect of a reaction")
                } else if (effect.variable is Action){
                    reactSignatureElement += ": Sched<" + getActionType(effect.variable as Action) + ">"
                    schedActionSet.add(effect.variable as Action)
                } else if (effect.variable is Port){
                    reactSignatureElement += ": ReadWrite<" + getPortType(effect.variable as Port) + ">"
                    if (effect.container === null) {
                        pr(reactEpilogue, "if (" + effect.variable.name + " !== undefined) {")
                        indent(reactEpilogue)
                        pr(reactEpilogue,  "__" + effect.variable.name + ".set(" + effect.variable.name + ");")
                        unindent(reactEpilogue)
                        pr(reactEpilogue, "}")
                    }
                }

                reactSignature.add(reactSignatureElement)

                functArg = "this." + generateVarRef(effect)
                if (effect.variable is Action){
                    reactFunctArgs.add("this.schedulable(" + functArg + ")")
                } else if (effect.variable is Port) {
                    reactFunctArgs.add("this.writable(" + functArg + ")")
                }

                if (effect.container === null) {
                    pr(reactPrologue, "let " + effect.variable.name + " = __" + effect.variable.name + ".get();")
                } else {
                    // Hierarchical references are handled later because there
                    // could be references to other members of the same reactor.
                    var args = containerToArgs.get(effect.container)
                    if (args === null) {
                        args = HashSet<Variable>();
                        containerToArgs.put(effect.container, args)
                    }
                    args.add(effect.variable)
                }
            }

            // Iterate through the actions to handle the prologue's
            // "actions" object
            var prologueActionObjectBody = StringJoiner(", ")
            for (act in schedActionSet) {
                prologueActionObjectBody.add(act.name + ": __" + act.name)
            }
            if (schedActionSet.size > 0) {
                pr(reactPrologue, "let actions = {"
                        + prologueActionObjectBody + "};")
            }

            // Add parameters to the react function
            for (param in reactor.parameters) {

                // Underscores are added to parameter names to prevent conflict with prologue
                reactSignature.add("__" + param.name + ": __Parameter<"
                        + param.targetType + ">")
                reactFunctArgs.add("this." + param.name)

                pr(reactPrologue, "let " + param.name + " = __" + param.name + ".get();")
            }

            // Add state to the react function
            for (state in reactor.stateVars) {
                // Underscores are added to state names to prevent conflict with prologue
                reactSignature.add("__" + state.name + ": __State<"
                        + getTargetType(state) + ">")
                reactFunctArgs.add("this." + state.name )

                pr(reactPrologue, "let " + state.name + " = __" + state.name + ".get();")
                pr(reactEpilogue, "if (" + state.name + " !== undefined) {")
                indent(reactEpilogue)
                pr(reactEpilogue,  "__" + state.name + ".set(" + state.name + ");")
                unindent(reactEpilogue)
                pr(reactEpilogue, "}")
            }

            // Initialize objects to enable hierarchical references.
            for (entry in containerToArgs.entries) {
                val initializer = StringJoiner(", ")
                for (variable in entry.value) {
                    initializer.add("""${variable.name}: __${entry.key.name}_${variable.name}.get()""")
                    if (variable is Input) {
                        pr(reactEpilogue,
                            """if (${entry.key.name}.${variable.name} !== undefined) {
                                    __${entry.key.name}_${variable.name}.set(${entry.key.name}.${variable.name})
                    }""")
                    }
                }
                pr(reactPrologue, """let ${entry.key.name} = {${initializer}}""")
            }

            // Assemble reaction triggers
            var reactionTriggers = StringJoiner(",\n")
            for (trigger in reaction.triggers) {
                if (trigger is VarRef) {
                    reactionTriggers.add("this." + generateVarRef(trigger))

                } else if (trigger.isStartup) {
                    reactionTriggers.add("this.startup")
                } else if (trigger.isShutdown) {
                    reactionTriggers.add("this.shutdown")
                }
            }

            // Write the reaction itself
            pr(reactorConstructor, "this.addReaction(")//new class<T> extends Reaction<T> {")
            indent(reactorConstructor)
            pr(reactorConstructor, "new __Triggers(" + reactionTriggers + "),")
            pr(reactorConstructor, "new __Args(" + reactFunctArgs + "),")
            pr(reactorConstructor, "function (" + reactSignature + ") {")
            indent(reactorConstructor)
            pr(reactorConstructor, "// =============== START react prologue")
            pr(reactorConstructor, reactPrologue)
            pr(reactorConstructor, "// =============== END react prologue")
            pr(reactorConstructor, "try {")
            indent(reactorConstructor)
            pr(reactorConstructor, reaction.code.toText())
            unindent(reactorConstructor)
            pr(reactorConstructor, "} finally {")
            indent(reactorConstructor)
            pr(reactorConstructor, "// =============== START react epilogue")
            pr(reactorConstructor, reactEpilogue)
            pr(reactorConstructor, "// =============== END react epilogue")
            unindent(reactorConstructor)
            pr(reactorConstructor, "}")
            unindent(reactorConstructor)
            if (reaction.deadline === null) {
                pr(reactorConstructor, "}")
            } else {
                pr(reactorConstructor, "},")
                var deadlineArgs = ""
                if (reaction.deadline.delay.parameter !== null) {
                    deadlineArgs += "this." + reaction.deadline.delay.parameter.name + ".get()";
                } else {
                    deadlineArgs += getTargetValue(reaction.deadline.delay)
                }
                pr(reactorConstructor, deadlineArgs + "," )
                pr(reactorConstructor, "function(" + reactSignature + ") {")
                indent(reactorConstructor)
                pr(reactorConstructor, "// =============== START deadline prologue")
                pr(reactorConstructor, reactPrologue)
                pr(reactorConstructor, "// =============== END deadline prologue")
                pr(reactorConstructor, "try {")
                indent(reactorConstructor)
                pr(reactorConstructor, toText(reaction.deadline.code))
                unindent(reactorConstructor)
                pr(reactorConstructor, "} finally {")
                indent(reactorConstructor)
                pr(reactorConstructor, "// =============== START deadline epilogue")
                pr(reactorConstructor, reactEpilogue)
                pr(reactorConstructor, "// =============== END deadline epilogue")
                unindent(reactorConstructor)
                pr(reactorConstructor, "}")
                unindent(reactorConstructor)
                pr(reactorConstructor, "}")
            }
            unindent(reactorConstructor)
            pr(reactorConstructor, ");")
        }
        ///////////////////// Reaction generation ends /////////////////////

        unindent(reactorConstructor)
        pr(reactorConstructor, "}")
        pr(reactorConstructor.toString())
        unindent()
        pr("}")
        pr("// =============== END reactor class " + reactor.name)
        pr("")

        return code.toString()
    }
}