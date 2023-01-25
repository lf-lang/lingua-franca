package org.lflang.generator.ts

import org.lflang.ErrorReporter
import org.lflang.ASTUtils
import org.lflang.federated.generator.FederateInstance
import org.lflang.generator.PrependOperator
import org.lflang.generator.getTargetTimeExpr
import org.lflang.isBank
import org.lflang.isMultiport
import org.lflang.joinWithCommas
import org.lflang.lf.*
import org.lflang.lf.Timer
import org.lflang.toText
import kotlin.collections.HashSet
import java.util.StringJoiner 
import java.util.LinkedList


/**
 * Reaction generator for TypeScript target.
 *
 *  @author Matt Weber
 *  @author Edward A. Lee
 *  @author Marten Lohstroh
 *  @author Christian Menard
 *  @author Hokeun Kim
 */
class TSReactionGenerator(
    private val errorReporter: ErrorReporter,
    private val reactor: Reactor
) {

    private fun VarRef.generateVarRef(): String {
        return if (this.container != null && this.container.isBank && this.variable is Port) {
            "this.${this.container.name}.port(it => it.${this.variable.name})"
        } else {
            "this.${ASTUtils.generateVarRef(this)}"
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
        val delay = reaction.deadline.delay
        deadlineArgs += if (delay is ParameterReference) {
            "this.${delay.parameter.name}.get()"
        } else {
            delay.toTsTime()
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
        reactFuncArgs: StringJoiner,
        reactSignature: StringJoiner
    ): String {
        // Assemble reaction triggers
        val reactionTriggers = StringJoiner(",\n")

        for (trigger in reaction.triggers) {
            if (trigger is VarRef) {
                reactionTriggers.add(trigger.generateVarRef())
            } else if (trigger is BuiltinTriggerRef) {
                when (trigger.type) {
                    BuiltinTrigger.STARTUP  -> reactionTriggers.add("this.startup")
                    BuiltinTrigger.SHUTDOWN -> reactionTriggers.add("this.shutdown")
                    else                    -> {}
                }
            }
        }

        return with(PrependOperator) {
            """
            |
            |this.add${if (reaction.isMutation) "Mutation" else "Reaction"}(
            |    new __Triggers($reactionTriggers),
            |    new __Args($reactFuncArgs),
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
        ${
                " |    "..if (reaction.deadline != null) generateDeadlineHandler(
                    reaction,
                    reactPrologue,
                    reactEpilogue,
                    reactSignature
                ) else "}"
            }
            |);
            |""".trimMargin()
        }
    }

    private fun generateReactionSignatureForTrigger(trigOrSource: VarRef): String {
        val reactSignatureElementType = when (trigOrSource.variable) {
            is Timer  -> "__Tag"
            is Action -> (trigOrSource.variable as Action).tsActionType
            is Port   -> (trigOrSource.variable as Port).tsPortType
            else      -> errorReporter.reportError("Invalid trigger: ${trigOrSource.variable.name}")
        }

        val portClassType = if (trigOrSource.variable.isMultiport) {
            "__InMultiPort<$reactSignatureElementType>"
        } else {
            "Read<$reactSignatureElementType>"
        }
        return if (trigOrSource.container != null && trigOrSource.container.isBank) {
            "${generateArg(trigOrSource)}: Array<$portClassType>"
        } else {
            "${generateArg(trigOrSource)}: $portClassType"
        }
    }

    private fun generateReactionSignatureElementForPortEffect(effect: VarRef, isMutation: Boolean): String {
        val outputPort = effect.variable as Port
        val portClassType = if (outputPort.isMultiport) {
            (if (isMutation) "__WritableMultiPort" else "MultiReadWrite") + "<${(effect.variable as Port).tsPortType}>"
        } else {
            (if (isMutation) "__WritablePort" else "ReadWrite") + "<${(effect.variable as Port).tsPortType}>"
        }

        return if (effect.container != null && effect.container.isBank) {
            "Array<$portClassType>"
        } else {
            portClassType
        }
    }

    private fun generateReactionEpilogueForPortEffect(effect: VarRef): String {
        val portEffect = effect.variable as Port
        val effectName = portEffect.name
        return if (effect.container == null) {
            if (portEffect.isMultiport) {
                """
                |$effectName.forEach((__element, __index) => {
                |    if (__element !== undefined) {
                |        __$effectName.set(__index, __element);
                |    }
                |});""".trimMargin()
            } else {
                """
                |if ($effectName !== undefined) {
                |    __$effectName.set($effectName);
                |}""".trimMargin()
            }
        } else {
            val containerName = effect.container.name
            if (effect.container.isBank) {
                if (portEffect.isMultiport) {
                    """
                    |$containerName.forEach((__reactor, __reactorIndex) => {
                    |   __reactor.$effectName.forEach((__element, __index) => {
                    |       if (__element !== undefined) {
                    |           __${containerName}_$effectName[__reactorIndex].set(__index, __element)
                    |       }
                    |   })
                    |});""".trimMargin()
                } else {
                    """
                    |$containerName.forEach((__reactor, __reactorIndex) => {
                    |   if (__reactor.$effectName !== undefined) {
                    |       __${containerName}_$effectName[__reactorIndex].set(__reactor.$effectName)
                    |   }
                    |});""".trimMargin()
                }
            } else {
                if (portEffect.isMultiport) {
                    """
                    |$containerName.$effectName.forEach((__element, __index) => {
                    |   if (__element !== undefined) {
                    |       __${containerName}_$effectName.set(__index, __element)
                    |   }
                    |});""".trimMargin()
                } else {
                    """
                    |if ($containerName.$effectName !== undefined) {
                    |    __${containerName}_$effectName.set($containerName.$effectName)
                    |}""".trimMargin()
                }
            }
        }
    }

    // TODO(hokeun): Decompose this function further.
    private fun generateSingleReaction(reactor: Reactor, reaction: Reaction): String {
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
        val triggersUnionSources = mutableSetOf<VarRef>().also {
            it.addAll(reaction.triggers.filterIsInstance<VarRef>())
            it.addAll(reaction.sources)
        }

        // Create a set of effect names so actions that appear
        // as both triggers/sources and effects can be
        // identified and added to the reaction arguments once.
        // We can't create a set of VarRefs because
        // an effect and a trigger/source with the same name are
        // unequal.
        // The key of the pair is the effect's container's name,
        // The effect of the pair is the effect's name
        val effectSet = reaction.effects.map {
            val key = it.container?.name ?: "" // The container
            val value = it.variable.name // The name of the effect
            key to value
        }.toMutableSet()


        // The prologue to the react function writes state
        // and parameters to local variables of the same name
        val reactPrologue = mutableListOf<String>()
        reactPrologue.add("const util = this.util;")

        // Add triggers and sources to the react function
        val containerToArgs = mutableMapOf<Instantiation, MutableSet<Variable>>()
        for (trigOrSource in triggersUnionSources) {
            // Actions that are both read and scheduled should only
            // appear once as a schedulable effect

            val trigOrSourceKey = trigOrSource.container?.name.orEmpty()
            val triggerName = trigOrSource.variable.name
            val trigOrSourcePair = trigOrSourceKey to triggerName

            if (trigOrSourcePair !in effectSet) {
                reactSignature.add(generateReactionSignatureForTrigger(trigOrSource))
                reactFunctArgs.add(trigOrSource.generateVarRef())
                if (trigOrSource.container == null) {
                    if (trigOrSource.variable.isMultiport) {
                        reactPrologue.add("let $triggerName = ${generateArg(trigOrSource)}.values();")
                    } else {
                        reactPrologue.add("let $triggerName = ${generateArg(trigOrSource)}.get();")
                    }
                } else {
                    val args = containerToArgs.computeIfAbsent(trigOrSource.container) { mutableSetOf() }
                    args.add(trigOrSource.variable)
                }
            }
        }
        val schedActionSet = mutableSetOf<Action>()

        // The epilogue to the react function writes local
        // state variables back to the state
        val reactEpilogue = mutableListOf<String>()
        for (effect in reaction.effects) {
            val reactSignatureElement = generateArg(effect)
            val functArg = effect.generateVarRef()
            when (val effectVar = effect.variable) {
                is Timer  -> {
                    errorReporter.reportError("A timer cannot be an effect of a reaction")
                }

                is Action -> {
                    reactSignature.add("$reactSignatureElement: Sched<${effectVar.tsActionType}>")
                    schedActionSet.add(effectVar)
                    reactFunctArgs.add("this.schedulable($functArg)")
                }

                is Port   -> {
                    val type = generateReactionSignatureElementForPortEffect(effect, reaction.isMutation)
                    reactSignature.add("$reactSignatureElement: $type")
                    reactEpilogue.add(generateReactionEpilogueForPortEffect(effect))
                    val funcArgs = if (effectVar.isMultiport) {
                        if (effect.container?.isBank == true) {
                            "this.${effect.container.name}.allWritable($functArg)"
                        } else {
                            "this.allWritable($functArg)"
                        }
                    } else {
                        if (effect.container?.isBank == true) {
                            "this.${effect.container.name}.writable($functArg)"
                        } else {
                            "this.writable($functArg)"
                        }
                    }
                    reactFunctArgs.add(funcArgs)
                }
            }

            if (effect.container == null) {
                if (effect.variable.isMultiport) {
                    val port = effect.variable as Port
                    reactPrologue.add("let ${port.name} = new Array<${port.tsPortType}>(__${port.name}.width());")
                } else {
                    reactPrologue.add("let ${effect.variable.name} = __${effect.variable.name}.get();")
                }
            } else {
                // Hierarchical references are handled later because there
                // could be references to other members of the same reactor.
                val args = containerToArgs.computeIfAbsent(effect.container) { HashSet() }
                args.add(effect.variable)
            }
        }

        // Iterate through the actions to handle the prologue's
        // "actions" object
        if (schedActionSet.size > 0) {
            val prologueActionObjectBody =
                schedActionSet.joinWithCommas { "${it.name}: __${it.name}" }
            reactPrologue.add("let actions = {$prologueActionObjectBody};")
        }

        // Add parameters to the react function
        for (param in reactor.parameters) {

            // Underscores are added to parameter names to prevent conflict with prologue
            val name = param.name
            reactSignature.add("__$name: __Parameter<${TSTypes.getTargetType(param)}>")
            reactFunctArgs.add("this.$name")
            reactPrologue.add("let $name = __$name.get();")
        }

        // Add state to the react function
        for (state in reactor.stateVars) {
            // Underscores are added to state names to prevent conflict with prologue
            val name = state.name
            reactSignature.add("__$name: __State<${TSTypes.getTargetType(state)}>")
            reactFunctArgs.add("this.$name")
            reactPrologue.add("let $name = __$name.get();")
            reactEpilogue.add(
                """
                    |if ($name !== undefined) {
                    |    __$name.set($name);
                    |}""".trimMargin()
            )
        }

        // Initialize objects to enable hierarchical references.
        for ((container, args) in containerToArgs.entries) {
            val containerName = container.name
            val initializer = args.joinWithCommas { variable ->
                "${variable.name}: __${containerName}_${variable.name}" +
                        // The parentheses are needed below to separate two if-else statements.
                        (if (container.isBank) "[i]" else "") +
                        if (variable.isMultiport) ".values()" else ".get()"
            }
            val prologuePart =
                if (container.isBank) {
                    """
                    |let $containerName = []
                    |for (let i = 0; i < ${container.widthSpec.toTSCode()}; i++) {
                    |   $containerName.push({$initializer})
                    |}""".trimMargin()
                } else {
                    "let $containerName = {$initializer}"
                }
            reactPrologue.add(prologuePart)
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

        ///////////////////// Reaction generation begins /////////////////////
        // TODO(hokeun): Consider separating this out as a new class.
        for (reaction in reactor.reactions) {
            // Write the reaction itself
            reactionCodes.add(generateSingleReaction(reactor, reaction))
        }
        return reactionCodes.joinToString("\n")
    }
}
