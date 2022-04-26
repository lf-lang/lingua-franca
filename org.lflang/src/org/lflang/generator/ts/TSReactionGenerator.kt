package org.lflang.generator.ts

import org.lflang.ErrorReporter
import org.lflang.ASTUtils
import org.lflang.federated.FederateInstance
import org.lflang.generator.PrependOperator
import org.lflang.isBank
import org.lflang.isMultiport
import org.lflang.lf.*
import org.lflang.lf.Timer
import org.lflang.toText
import java.util.*
import kotlin.collections.HashSet

/**
 * Reaction generator for TypeScript target.
 *
 *  @author{Matt Weber <matt.weber@berkeley.edu>}
 *  @author{Edward A. Lee <eal@berkeley.edu>}
 *  @author{Marten Lohstroh <marten@berkeley.edu>}
 *  @author {Christian Menard <christian.menard@tu-dresden.de>}
 *  @author {Hokeun Kim <hokeunkim@berkeley.edu>}
 */
class TSReactionGenerator(
    // TODO(hokeun): Remove dependency on TSGenerator.
    private val tsGenerator: TSGenerator,
    private val errorReporter: ErrorReporter,
    private val reactor : Reactor,
    private val federate: FederateInstance
) {
    private fun Value.getTargetValue(): String = tsGenerator.getTargetValueW(this)
    private fun Parameter.getTargetType(): String = tsGenerator.getTargetTypeW(this)
    private fun StateVar.getTargetType(): String = tsGenerator.getTargetTypeW(this)
    private fun Type.getTargetType(): String = tsGenerator.getTargetTypeW(this)

    private fun VarRef.generateVarRef(): String {
        return if (this.container != null && this.container.isBank && this.variable is Port) {
            "this.${this.container.name}.port(it => it.${this.variable.name})"
        } else {
            "this.${ASTUtils.generateVarRef(this)}"
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
        if (port.type != null) {
            return port.type.getTargetType()
        } else {
            return "Present"
        }
    }

    private fun generateArg(v: VarRef): String {
        return if (v.container != null) {
            "__${v.container.name}_${v.variable.name}"
        } else {
            "__${v.variable.name}"
        }
    }

    private fun generateDeadlineHandler(
        reaction: Reaction,
        reactPrologue: String,
        reactEpilogue: String,
        reactSignature: StringJoiner
    ): String {
        var deadlineArgs = ""
        if (reaction.deadline.delay.parameter != null) {
            deadlineArgs += "this.${reaction.deadline.delay.parameter.name}.get()";
        } else {
            deadlineArgs += reaction.deadline.delay.getTargetValue()
        }

        return with(PrependOperator) {
            """
            |},
            |$deadlineArgs,
            |function($reactSignature) {
            |    // =============== START deadline prologue
        ${" |    "..reactPrologue}
            |    // =============== END deadline prologue
            |    try {
        ${" |        "..reaction.deadline.code.toText()}
            |    } finally {
            |        // =============== START deadline epilogue
        ${" |        "..reactEpilogue}
            |        // =============== END deadline epilogue
            |    }
            |}
        """.trimMargin()
        }

    }

    private fun generateReactionString(
        reaction: Reaction,
        reactPrologue: String,
        reactEpilogue: String,
        reactFunctArgs: StringJoiner,
        reactSignature: StringJoiner
    ): String {
        // Assemble reaction triggers
        val reactionTriggers = StringJoiner(",\n")
        for (trigger in reaction.triggers) {
            if (trigger is VarRef) {
                reactionTriggers.add(trigger.generateVarRef())
            } else if (trigger.isStartup) {
                reactionTriggers.add("this.startup")
            } else if (trigger.isShutdown) {
                reactionTriggers.add("this.shutdown")
            }
        }
        return with(PrependOperator) {
            """
            |
            |this.addReaction(
            |    new __Triggers($reactionTriggers),
            |    new __Args($reactFunctArgs),
            |    function ($reactSignature) {
            |        // =============== START react prologue
        ${" |        "..reactPrologue}
            |        // =============== END react prologue
            |        try {
        ${" |            "..reaction.code.toText()}
            |        } finally {
            |            // =============== START react epilogue
        ${" |            "..reactEpilogue}            
            |            // =============== END react epilogue
            |        }
        ${" |    "..if (reaction.deadline != null) generateDeadlineHandler(reaction, reactPrologue, reactEpilogue, reactSignature) else "}"}
            |);
        """.trimMargin()
        }
    }

    private fun generateReactionSignatureForTrigger(trigOrSource: VarRef): String {
        var reactSignatureElementType = if (trigOrSource.variable.name.startsWith("networkMessage")) {
            // Special handling for the networkMessage action created by
            // FedASTUtils.makeCommunication(), by assigning TypeScript
            // Buffer type for the action. Action<Buffer> is used as
            // FederatePortAction in federation.ts.
            "Buffer"
        } else if (trigOrSource.variable is Timer) {
            "__Tag"
        } else if (trigOrSource.variable is Action) {
            getActionType(trigOrSource.variable as Action, federate)
        } else if (trigOrSource.variable is Port) {
            getPortType(trigOrSource.variable as Port)
        } else {
            errorReporter.reportError("Invalid trigger: ${trigOrSource.variable.name}")
        }

        val portClassType = if (trigOrSource.variable.isMultiport) {
            "__InMultiPort<${reactSignatureElementType}>"
        } else {
            "Read<${reactSignatureElementType}>"
        }
        return if (trigOrSource.container != null && trigOrSource.container.isBank) {
            "${generateArg(trigOrSource)}: Array<$portClassType>"
        } else {
            "${generateArg(trigOrSource)}: $portClassType"
        }
    }

    private fun generateReactionSignatureElementForPortEffect(effect: VarRef): String {
        val outputPort = effect.variable as Port
        val portClassType = if (outputPort.isMultiport) {
            "MultiReadWrite<${getPortType(effect.variable as Port)}>"
        } else {
            "ReadWrite<${getPortType(effect.variable as Port)}>"
        }

        return if (effect.container != null && effect.container.isBank) {
            "Array<${portClassType}>"
        } else {
            portClassType
        }
    }

    private fun generateReactionEpilogueForPortEffect(effect: VarRef): String {
        val portEffect = effect.variable as Port
        if (effect.container == null) {
            if (portEffect.isMultiport) {
                return """
                    |${portEffect.name}.forEach((__element, __index) => {
                    |    if (__element !== undefined) {
                    |        __${portEffect.name}.set(__index, __element);
                    |    }
                    |});""".trimMargin()
            } else {
                return """
                    |if (${portEffect.name} !== undefined) {
                    |    __${portEffect.name}.set(${portEffect.name});
                    |}""".trimMargin()
            }
        } else {
            if (effect.container.isBank) {
                if (portEffect.isMultiport) {
                    return """
                    |${effect.container.name}.forEach((__reactor, __reactorIndex) => {
                    |   __reactor.${portEffect.name}.forEach((__element, __index) => {
                    |       if (__element !== undefined) {
                    |           __${effect.container.name}_${portEffect.name}[__reactorIndex].set(__index, __element)
                    |       }
                    |   })
                    |});""".trimMargin()
                } else {
                    return """
                    |${effect.container.name}.forEach((__reactor, __reactorIndex) => {
                    |   if (__reactor.${portEffect.name} !== undefined) {
                    |       __${effect.container.name}_${portEffect.name}[__reactorIndex].set(__reactor.${portEffect.name})
                    |   }
                    |});""".trimMargin()
                }
            } else {
                if (portEffect.isMultiport) {
                    return """
                    |${effect.container.name}.${portEffect.name}.forEach((__element, __index) => {
                    |   if (__element !== undefined) {
                    |       __${effect.container.name}_${portEffect.name}.set(__index, __element)
                    |   }
                    |});""".trimMargin()
                } else {
                    return """
                    |if (${effect.container.name}.${portEffect.name} !== undefined) {
                    |    __${effect.container.name}_${portEffect.name}.set(${effect.container.name}.${portEffect.name})
                    |}""".trimMargin()
                }
            }
        }
    }

    // TODO(hokeun): Decompose this function further.
    private fun generateSingleReaction(reactor : Reactor, reaction: Reaction): String {
        // Determine signature of the react function
        val reactSignature = StringJoiner(", ")
        reactSignature.add("this")

        // Assemble react function arguments from sources and effects
        // Arguments are either elements of this reactor, or an object
        // representing a contained reactor with properties corresponding
        // to listed sources and effects.

        // If a source or effect is an element of this reactor, add it
        // directly to the reactFunctArgs string. If it isn't, write it
        // into the containerToArgs map, and add it to the string later.
        val reactFunctArgs = StringJoiner(", ")
        // Combine triggers and sources into a set
        // so we can iterate over their union
        val triggersUnionSources = HashSet<VarRef>()
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
        val effectSet = HashSet<Pair<String, String>>()

        for (effect in reaction.effects) {
            var key = ""; // The container, defaults to an empty string
            val value = effect.variable.name; // The name of the effect
            if (effect.container != null) {
                key = effect.container.name
            }
            effectSet.add(Pair(key, value))
        }

        // The prologue to the react function writes state
        // and parameters to local variables of the same name
        val reactPrologue = LinkedList<String>()
        reactPrologue.add("const util = this.util;")

        // Add triggers and sources to the react function
        val containerToArgs = HashMap<Instantiation, HashSet<Variable>>();
        for (trigOrSource in triggersUnionSources) {
            // Actions that are both read and scheduled should only
            // appear once as a schedulable effect

            var trigOrSourceKey = "" // The default for no container
            val trigOrSourceValue = trigOrSource.variable.name
            if (trigOrSource.container != null) {
                trigOrSourceKey = trigOrSource.container.name
            }
            val trigOrSourcePair = Pair(trigOrSourceKey, trigOrSourceValue)

            if (!effectSet.contains(trigOrSourcePair)) {
                reactSignature.add(generateReactionSignatureForTrigger(trigOrSource))
                reactFunctArgs.add(trigOrSource.generateVarRef())
                if (trigOrSource.container == null) {
                    if (trigOrSource.variable.isMultiport) {
                        val inputPort = trigOrSource.variable as Port
                        reactPrologue.add(
                            "let ${inputPort.name} = ${generateArg(trigOrSource)}.values();")
                    } else {
                        reactPrologue.add("let ${trigOrSource.variable.name} = ${generateArg(trigOrSource)}.get();")
                    }
                } else {
                    var args = containerToArgs.get(trigOrSource.container)
                    if (args == null) {
                        // Create the HashSet for the container
                        // and handle it later.
                        args = HashSet<Variable>()
                        containerToArgs.put(trigOrSource.container, args)
                    }
                    args.add(trigOrSource.variable)
                }
            }
        }
        val schedActionSet = HashSet<Action>()

        // The epilogue to the react function writes local
        // state variables back to the state
        val reactEpilogue = LinkedList<String>()
        for (effect in reaction.effects) {
            var reactSignatureElement = generateArg(effect)
            if (effect.variable is Timer) {
                errorReporter.reportError("A timer cannot be an effect of a reaction")
            } else if (effect.variable is Action){
                reactSignatureElement += ": Sched<" + getActionType(effect.variable as Action, federate) + ">"
                schedActionSet.add(effect.variable as Action)
            } else if (effect.variable is Port){
                reactSignatureElement += ": ${generateReactionSignatureElementForPortEffect(effect)}"
                reactEpilogue.add(generateReactionEpilogueForPortEffect(effect))
            }

            reactSignature.add(reactSignatureElement)

            var functArg = effect.generateVarRef()
            if (effect.variable is Action){
                reactFunctArgs.add("this.schedulable($functArg)")
            } else if (effect.variable is Port) {
                val port = effect.variable as Port
                if (port.isMultiport) {
                    if (effect.container != null && effect.container.isBank) {
                        reactFunctArgs.add("this.${effect.container.name}.allWritable($functArg)")
                    } else {
                        reactFunctArgs.add("this.allWritable($functArg)")
                    }
                } else {
                    if (effect.container != null && effect.container.isBank) {
                        reactFunctArgs.add("this.${effect.container.name}.writable($functArg)")
                    } else {
                        reactFunctArgs.add("this.writable($functArg)")
                    }
                }
            }

            if (effect.container == null) {
                if (effect.variable.isMultiport) {
                    val port = effect.variable as Port
                    reactPrologue.add("let ${port.name} = new Array<${getPortType(port)}>(__${port.name}.width());")
                } else {
                    reactPrologue.add("let ${effect.variable.name} = __${effect.variable.name}.get();")
                }
            } else {
                // Hierarchical references are handled later because there
                // could be references to other members of the same reactor.
                var args = containerToArgs.get(effect.container)
                if (args == null) {
                    args = HashSet<Variable>();
                    containerToArgs.put(effect.container, args)
                }
                args.add(effect.variable)
            }
        }

        // Iterate through the actions to handle the prologue's
        // "actions" object
        if (schedActionSet.size > 0) {
            val prologueActionObjectBody = StringJoiner(", ")
            for (act in schedActionSet) {
                prologueActionObjectBody.add("${act.name}: __${act.name}")
            }
            reactPrologue.add("let actions = {$prologueActionObjectBody};")
        }

        // Add parameters to the react function
        for (param in reactor.parameters) {

            // Underscores are added to parameter names to prevent conflict with prologue
            reactSignature.add("__${param.name}: __Parameter<${param.getTargetType()}>")
            reactFunctArgs.add("this.${param.name}")

            reactPrologue.add("let ${param.name} = __${param.name}.get();")
        }

        // Add state to the react function
        for (state in reactor.stateVars) {
            // Underscores are added to state names to prevent conflict with prologue
            reactSignature.add("__${state.name}: __State<${state.getTargetType()}>")
            reactFunctArgs.add("this.${state.name}")

            reactPrologue.add("let ${state.name} = __${state.name}.get();")
            reactEpilogue.add(with(PrependOperator) {"""
                    |if (${state.name} !== undefined) {
                    |    __${state.name}.set(${state.name});
                    |}""".trimMargin()})
        }

        // Initialize objects to enable hierarchical references.
        for (entry in containerToArgs.entries) {
            val initializer = StringJoiner(", ")
            for (variable in entry.value) {
                initializer.add("${variable.name}: __${entry.key.name}_${variable.name}" +
                        // The parentheses are needed below to separate two if-else statements.
                        (if (entry.key.isBank) "[i]" else "") +
                        if (variable.isMultiport) ".values()" else ".get()")
            }
            if (entry.key.isBank) {
                reactPrologue.add(
                    """
                        |let ${entry.key.name} = []
                        |for (let i = 0; i < ${entry.key.widthSpec.toTSCode()}; i++) {
                        |   ${entry.key.name}.push({${initializer}})
                        |}""".trimMargin()
                )
            } else {
                reactPrologue.add("let ${entry.key.name} = {${initializer}}")
            }
        }

        // Generate reaction as a formatted string.
        return generateReactionString(
            reaction,
            reactPrologue.joinToString("\n"),
            reactEpilogue.joinToString("\n"),
            reactFunctArgs,
            reactSignature
        )
    }

    fun generateAllReactions(): String {
        val reactionCodes = LinkedList<String>()
        // Next handle reaction instances.
        // If the app is federated, only generate
        // reactions that are contained by that federate
        val generatedReactions: List<Reaction>
        if (reactor.isFederated) {
            generatedReactions = LinkedList<Reaction>()
            for (reaction in reactor.reactions) {
                // TODO(hokeun): Find a better way to gracefully handle this skipping.
                // Do not add reactions created by generateNetworkOutputControlReactionBody
                // or generateNetworkInputControlReactionBody.
                if (reaction.code.toText().contains("generateNetworkOutputControlReactionBody")
                    || reaction.code.toText().contains("generateNetworkInputControlReactionBody")) {
                    continue;
                }
                if (federate.contains(reaction)) {
                    generatedReactions.add(reaction)
                }
            }
        } else {
            generatedReactions = reactor.reactions
        }

        ///////////////////// Reaction generation begins /////////////////////
        // TODO(hokeun): Consider separating this out as a new class.
        for (reaction in generatedReactions) {
            // Write the reaction itself
            reactionCodes.add(generateSingleReaction(reactor, reaction))
        }
        return reactionCodes.joinToString("\n")
    }
}