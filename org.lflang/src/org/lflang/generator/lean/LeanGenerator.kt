package org.lflang.generator.lean

import org.eclipse.emf.ecore.resource.Resource
import org.lflang.*
import org.lflang.Target
import org.lflang.generator.*
import org.lflang.generator.PrependOperator.rangeTo
import org.lflang.lf.*
import org.lflang.lf.Timer
import org.lflang.scoping.LFGlobalScopeProvider
import org.lflang.util.FileUtil
import java.nio.file.Path
import java.util.*

class LeanGenerator(
    val context: LFGeneratorContext,
    errorReporter: ErrorReporter,
    private val scopeProvider: LFGlobalScopeProvider
) :
    GeneratorBase(context) {

    companion object {
        /** Path to the Lean runtime library (relative to class path)  */
        const val runtimeDir = "/lib/lean/reactor-lean"
        const val runtimeName = "reactor-lean"
    }

    override fun doGenerate(resource: Resource, context: LFGeneratorContext) {
        super.doGenerate(resource, context)

        if (!GeneratorUtils.canGenerate(errorsOccurred(), mainDef, errorReporter, context)) return

        FileUtil.copyDirectoryFromClassPath(runtimeDir, context.fileConfig.srcGenPath, false)

        // We need to generate two files:
        // `Main.lean`: The (only) file to which we emit code and which contains the entry point of the program.
        // `lakefile.lean`: The configuration file for the project required by Lean's "Lake" package manager.
        val mainFilePath = context.fileConfig.srcGenPath.resolve( "Main.lean")
        val lakefilePath = context.fileConfig.srcGenPath.resolve( "lakefile.lean")

        val mainFile = genMain(reactors)
        val lakefile = genLakefile(mainDef.name)

        FileUtil.writeToFile(mainFile, mainFilePath, true)
        FileUtil.writeToFile(lakefile, lakefilePath, true)

        invokeLeanCompiler(context, mainDef.name, emptyMap())
    }

    private fun genParameter(param: Parameter): String {
        val type = param.type.code?.let { ASTUtils.toOriginalText(it) } ?: LeanTypes.getTargetType(param.type)
        return "${param.name} : ${type} := ${LeanTypes.getTargetInitializer(param.init, param.type, true)}"
    }

    private fun genTypedVar(v: TypedVariable): String {
        val type = v.type.code?.let { ASTUtils.toOriginalText(it) } ?: LeanTypes.getTargetType(v.type)
        return "${v.name} : ${type}"
    }

    private fun genState(stateVar: StateVar): String {
        val default = stateVar.init?.let {
            " := ${LeanTypes.getTargetInitializer(it, stateVar.type, true)}"
        } ?: ""

        val type = stateVar.type.code?.let { ASTUtils.toOriginalText(it) } ?: LeanTypes.getTargetType(stateVar.type)
        return "${stateVar.name} : ${type}$default"
    }

    private fun genTimer(timer: Timer): String {
        val offset = if (timer.offset == null) "0" else LeanTypes.getTargetExpr(timer.offset, InferredType.time())
        val period = if (timer.period == null) "0" else LeanTypes.getTargetExpr(timer.period, InferredType.time())
        return """
        |{
        |  name   ${timer.name}
        |  offset $offset
        |  period $period
        |}
        """.trimMargin()
    }

    private fun genNested(nested: Instantiation): String {
        val params = nested.parameters.joinWithCommas(trailing = false) { a ->
            val type = a.lhs.type.code?.let { ASTUtils.toOriginalText(it) } ?: LeanTypes.getTargetType(a.lhs.type)
            "${a.lhs.name} : ${type} := ${LeanTypes.getTargetInitializer(a.rhs, a.lhs.type, true)}"
        }
        return "${nested.name} : ${nested.reactorClass.name} := [$params]"
    }

    private fun genConnection(connection: Connection): String {
        return connection.leftPorts.flatMap { lhs ->
            val src = "${lhs.container.name}.${lhs.variable.name}"
            connection.rightPorts.map { rhs ->
                val dst = "${rhs.container.name}.${rhs.variable.name}"
                return connection.delay?.let {
                    "$src : $dst := ${LeanTypes.getTargetExpr(it, InferredType.time())}"
                } ?: "$src : $dst"
            }
        }
        .joinWithCommas(trailing = false)
    }

    private fun genReactionPortSources(reaction: Reaction) =
        (reaction.sources + reaction.triggers).mapNotNull { t ->
            if (t is VarRef && t.variable is Port) {
                if (t.container == null) {
                    t.variable.name
                } else {
                    "${t.container.name}.${t.variable.name}"
                }
            } else {
                null
            }
        }

    private fun genReactionActionSources(reaction: Reaction) =
        (reaction.sources + reaction.triggers).mapNotNull { t ->
            if (t is VarRef && t.variable is Action) t.variable.name else null
        }

    private fun genReactionPortEffects(reaction: Reaction) =
        reaction.effects.mapNotNull {
            if (it.container == null) {
                (it.variable as? Port)?.name
            } else {
                "${it.container.name}.${(it.variable as? Port)?.name}"
            }
        }

    private fun genReactionActionEffects(reaction: Reaction) =
        reaction.effects.mapNotNull { (it.variable as? Action)?.name }

    private fun genReactionPortTriggers(reaction: Reaction) =
        reaction.triggers.mapNotNull { t ->
            if (t is VarRef && t.variable is Port) {
                if (t.container == null) {
                    t.variable.name
                } else {
                    "${t.container.name}.${t.variable.name}"
                }
            } else {
                null
            }
        }

    private fun genReactionActionTriggers(reaction: Reaction) =
        reaction.triggers.mapNotNull { t ->
            if (t is VarRef && t.variable is Action) t.variable.name else null
        }

    private fun genReactionTimerTriggers(reaction: Reaction) =
        reaction.triggers.mapNotNull { t ->
            if (t is VarRef && t.variable is Timer) t.variable.name else null
        }

    private fun genReactionMetaTriggers(reaction: Reaction) = reaction.triggers.mapNotNull { t ->
        when {
            t is BuiltinTriggerRef && t.type == BuiltinTrigger.STARTUP  -> "startup"
            t is BuiltinTriggerRef && t.type == BuiltinTrigger.SHUTDOWN -> "shutdown"
            else                                                        -> null
        }
    }

    private fun genReactionKind(reaction: Reaction) =
        if (reaction.attributes.any { it.attrName == "io" })
            "impure"
        else
            "pure"

    private fun genReaction(reaction: Reaction): String {
        return """
           |{
           |  kind          ${genReactionKind(reaction)}
           |  portSources   ${genReactionPortSources(reaction)}
           |  portEffects   ${genReactionPortEffects(reaction)}
           |  actionSources ${genReactionActionSources(reaction)}
           |  actionEffects ${genReactionActionEffects(reaction)}
           |  triggers {
           |     ports   ${genReactionPortTriggers(reaction)}
           |     actions ${genReactionActionTriggers(reaction)}
           |     timers  ${genReactionTimerTriggers(reaction)}
           |     meta    ${genReactionMetaTriggers(reaction)}
           |  }
           |  body {
        ${"|    "..(ASTUtils.toOriginalText(reaction.code))}
           |  }
           |}
        """.trimMargin()
    }

    private fun genNesteds(nested: List<Instantiation>): String {
        return if (nested.isEmpty()) {
            "nested      []"
        } else {
            """
               |nested      [
            ${"|  "..(nested.joinToString(",\n") { genNested(it)})}
               |]
            """.trimMargin()
        }
    }

    private fun genReactions(reactions: List<Reaction>): String {
        return if (reactions.isEmpty()) {
            "reactions   []"
        } else {
            """
               |reactions   [
            ${"|  "..(reactions.joinToString(",\n") { genReaction(it)})}
               |]
            """.trimMargin()
        }
    }

    private fun genTimers(timers: List<Timer>): String {
        return if (timers.isEmpty()) {
            "timers      []"
        } else {
            """
               |timers      [
            ${"|  "..(timers.joinToString(",\n") { genTimer(it)})}
               |]
            """.trimMargin()
        }
    }

    private fun genReactor(reactor: Reactor): String {
        val logicalActions = reactor.actions.filter { it.isLogical }
        return with(PrependOperator) {
            """
               |reactor ${reactor.name}
               |  parameters  [${reactor.parameters.joinWithCommas(trailing = false) { genParameter(it) }}]
               |  inputs      [${reactor.inputs.joinWithCommas(trailing = false)     { genTypedVar(it) }}]
               |  outputs     [${reactor.outputs.joinWithCommas(trailing = false)    { genTypedVar(it) }}]
               |  actions     [${logicalActions.joinWithCommas(trailing = false)     { genTypedVar(it) }}]
               |  state       [${reactor.stateVars.joinWithCommas(trailing = false)  { genState(it) }}]
            ${"|  "..(genTimers(reactor.timers))}
            ${"|  "..(genNesteds(reactor.instantiations))}
               |  connections [${reactor.connections.joinWithCommas(trailing = false) { genConnection(it) }}]
            ${"|  "..(genReactions(reactor.reactions))}
            """.trimMargin()
        }
    }

    // BUG: This always returns an empty list.
    private fun genSchedule(): String {
        main = ReactorInstance(ASTUtils.toDefinition(mainDef.reactorClass), errorReporter)
        return ReactionInstanceGraph(main)
            .nodesInReverseTopologicalOrder()
            .reversed()
            .joinToString(",\n") {
                val reactorId = it.reaction.parent.fullName.dropWhile { it != '.' }
                "${reactorId.drop(1)}._${it.reaction.index}"
            }
    }

    private fun genPreamble(): String {
        val preambles = resources.flatMap {
            it.eResource.model.preambles.map {
                ASTUtils.toOriginalText(it.code)
            }
        }

        return if (preambles.isEmpty()) {
            "\n\n"
        } else {
            "\n\n${preambles.joinLn()}\n\n"
        }
    }

    private fun genEpilogue() =
        resources
            .flatMap {
                it.eResource.model.epilogues.map {
                    ASTUtils.toOriginalText(it.code)
                }
            }.joinLn()

    private fun genLFBlock(reactors: List<Reactor>): String {
        // Moves the main reactor to the front of the list.
        Collections.swap(reactors, 0, reactors.indexOfFirst { it.isMain })

        return with(PrependOperator) {
            """
                |lf {
             ${"|  "..(reactors.joinToString("\n\n") { genReactor(it) })}
                |
                |  schedule [
             ${"|    "..(genSchedule())}
                |  ]
                |}
            """.trimMargin()
        }
    }

    private fun genMain(reactors: List<Reactor>): String {
        return with(PrependOperator) {
            """
               |import Runtime
            ${"|"..(genPreamble())}
            ${"|"..(genLFBlock(reactors))}
               |
               |open LF
            ${"|"..(genEpilogue())}
            """.trimMargin()
        }
    }

    private fun genLakefile(exeName: String) =
        """
        |import Lake
        |open Lake DSL
        |
        |package reactor_lean
        |
        |lean_lib Runtime
        |
        |@[default_target]
        |lean_exe Executable {
        |  root := `Main
        |  exeName := "$exeName"
        |}
        """.trimMargin()

    private fun invokeLeanCompiler(context: LFGeneratorContext, executableName: String, codeMaps: Map<Path, CodeMap>) {
        val lakeUpdateCommand = commandFactory.createCommand("lake", listOf("update"), context.fileConfig.srcGenPath.toAbsolutePath())
        lakeUpdateCommand.run()

        val buildCommand = commandFactory.createCommand("lake", listOf("build"), context.fileConfig.srcGenPath.toAbsolutePath())
        val returnCode = buildCommand.run()

        if (returnCode == 0) {
            println("SUCCESS (compiling generated Lean code)")

            // Copies the executable from the directory into which Lake places it, to the proper binary path.
            val lakeBinPath = context.fileConfig.srcGenBasePath.resolve("${mainDef.name}/build/bin/${mainDef.name}")
            FileUtil.copyFile(lakeBinPath, context.fileConfig.binPath)

            context.finish(GeneratorResult(GeneratorResult.Status.COMPILED, context, codeMaps))
        } else if (context.cancelIndicator.isCanceled) {
            context.finish(GeneratorResult.CANCELLED)
        } else {
            if (!errorsOccurred()) errorReporter.reportError(
                "lake failed with error code $returnCode and reported the following error(s):\n${buildCommand.errors}"
            )
            context.finish(GeneratorResult.FAILED)
        }
    }

    override fun getTarget() = Target.Lean

    override fun getTargetTypes(): TargetTypes = LeanTypes
}
