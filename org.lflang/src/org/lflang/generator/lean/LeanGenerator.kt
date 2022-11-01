package org.lflang.generator.lean

import org.eclipse.emf.ecore.resource.Resource
import org.lflang.ErrorReporter
import org.lflang.FileConfig
import org.lflang.InferredType
import org.lflang.Target
import org.lflang.TargetProperty
import org.lflang.baseType
import org.lflang.generator.CodeMap
import org.lflang.generator.GeneratorBase
import org.lflang.generator.GeneratorResult
import org.lflang.generator.GeneratorUtils
import org.lflang.generator.LFGeneratorContext
import org.lflang.generator.PrependOperator
import org.lflang.generator.PrependOperator.rangeTo
import org.lflang.generator.TargetTypes
import org.lflang.generator.cpp.CppTypes
import org.lflang.generator.cpp.name
import org.lflang.generator.cpp.toTime
import org.lflang.isLogical
import org.lflang.joinLn
import org.lflang.joinWithCommas
import org.lflang.lf.Action
import org.lflang.lf.ActionOrigin
import org.lflang.lf.BuiltinTrigger
import org.lflang.lf.BuiltinTriggerRef
import org.lflang.lf.Connection
import org.lflang.lf.Expression
import org.lflang.lf.Input
import org.lflang.lf.Instantiation
import org.lflang.lf.Output
import org.lflang.lf.Parameter
import org.lflang.lf.ParameterReference
import org.lflang.lf.Port
import org.lflang.lf.Reaction
import org.lflang.lf.Reactor
import org.lflang.lf.StateVar
import org.lflang.lf.Time
import org.lflang.lf.Timer
import org.lflang.lf.TriggerRef
import org.lflang.lf.TypedVariable
import org.lflang.lf.VarRef
import org.lflang.scoping.LFGlobalScopeProvider
import org.lflang.toText
import org.lflang.toTextTokenBased
import org.lflang.util.FileUtil
import java.nio.file.Files
import java.nio.file.Path
import java.util.*

class LeanGenerator(
    val fileConfig: LeanFileConfig,
    errorReporter: ErrorReporter,
    private val scopeProvider: LFGlobalScopeProvider
) :
    GeneratorBase(fileConfig, errorReporter) {

    companion object {
        /** Path to the Lean runtime library (relative to class path)  */
        const val runtimeDir = "/lib/lean/reactor-lean"
        const val runtimeName = "reactor-lean"
    }

    override fun doGenerate(resource: Resource, context: LFGeneratorContext) {
        super.doGenerate(resource, context)

        if (!GeneratorUtils.canGenerate(errorsOccurred(), mainDef, errorReporter, context)) return

        Files.createDirectories(fileConfig.srcGenPath)
        FileUtil.copyDirectoryFromClassPath(runtimeDir, fileConfig.srcGenPath, true)
        val mainFilePath = fileConfig.srcGenPath.resolve( "Main.lean")

        val mainFile = genMain(reactors)
        FileUtil.writeToFile(mainFile, mainFilePath, true)

        invokeLeanCompiler(context, "Main", emptyMap())
    }

    private fun genParameter(param: Parameter) =
        "${param.name} : ${LeanTypes.getTargetType(param.type)} := ${LeanTypes.getTargetInitializer(param.init, param.type, param.braces.isNotEmpty())}"

    private fun genTypedVar(v: TypedVariable) =
        "${v.name} : ${LeanTypes.getTargetType(v.type)}"

    private fun genState(stateVar: StateVar): String {
        val default = LeanTypes.getTargetInitializer(stateVar.init, stateVar.type, stateVar.braces.isNotEmpty())
        val defaultStr = default.let { " := $it" } ?: ""
        return "${stateVar.name} : ${LeanTypes.getTargetType(stateVar.type)}$defaultStr"
    }

    private fun genTimer(timer: Timer) = """
        |{
        |  name   ${timer.name}
        |  offset ${LeanTypes.getTargetExpr(timer.offset, InferredType.time())}
        |  period ${LeanTypes.getTargetExpr(timer.period, InferredType.time())}
        |}
        """.trimMargin()

    private fun genNested(nested: Instantiation): String {
        val params = nested.parameters.joinWithCommas(trailing = false) { a ->
            "${a.lhs.name} : ${a.lhs.type.baseType} := ${LeanTypes.getTargetInitializer(a.rhs, a.lhs.type, a.lhs.braces.isNotEmpty())}"
        }
        return "${nested.name} : ${nested.reactorClass.name} := [$params]"
    }

    private fun genConnection(connection: Connection) =
        "${connection.leftPorts.first().container.name}.${connection.leftPorts.first().variable.name} : ${connection.rightPorts.first().container.name}.${connection.rightPorts.first().variable.name}"

    private fun genReactionPortSources(reaction: Reaction) =
        (reaction.sources + reaction.triggers).mapNotNull { t ->
            if (t is VarRef && t.variable is Port) t.variable.name else null
        }

    private fun genReactionActionSources(reaction: Reaction) =
        (reaction.sources + reaction.triggers).mapNotNull { t ->
            if (t is VarRef && t.variable is Action) t.variable.name else null
        }

    private fun genReactionPortEffects(reaction: Reaction) =
        reaction.effects.mapNotNull { (it.variable as? Port)?.name }

    private fun genReactionActionEffects(reaction: Reaction) =
        reaction.effects.mapNotNull { (it.variable as? Action)?.name }

    private fun genReactionPortTriggers(reaction: Reaction) =
        reaction.triggers.mapNotNull { t ->
            if (t is VarRef && t.variable is Port) t.variable.name else null
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

    private fun genReaction(reaction: Reaction): String {
        return """
           |{
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
        ${"|    "..(reaction.code.toTextTokenBased()?.removeSurrounding("{=", "=}") ?: "") /*HACK*/}
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

    private fun genLFBlock(reactors: List<Reactor>): String {
        Collections.swap(reactors, 0, reactors.indexOfFirst { it.isMain }) // Move the main reactor to the front of the list.
        return with(PrependOperator) {
            """
                |lf {
             ${"|  "..(reactors.joinToString("\n\n") { genReactor(it) })}
                |}
            """.trimMargin()
        }
    }

    private fun genMain(reactors: List<Reactor>): String {
        return """
           |import Runtime
           | 
        ${"|"..(genLFBlock(reactors))} 
        """.trimMargin()
    }

    private fun invokeLeanCompiler(context: LFGeneratorContext, executableName: String, codeMaps: Map<Path, CodeMap>) {
        val lakeCommand = commandFactory.createCommand("lake", listOf("build"), fileConfig.srcGenPath.toAbsolutePath())
        val returnCode = lakeCommand.run()

        if (returnCode == 0) {
            println("SUCCESS (compiling generated Lean code)")
            context.finish(GeneratorResult.Status.COMPILED, executableName, fileConfig, codeMaps)
        } else if (context.cancelIndicator.isCanceled) {
            context.finish(GeneratorResult.CANCELLED)
        } else {
            if (!errorsOccurred()) errorReporter.reportError(
                "lake failed with error code $returnCode and reported the following error(s):\n${lakeCommand.errors}"
            )
            context.finish(GeneratorResult.FAILED)
        }
    }

    override fun getTarget() = Target.Lean

    override fun getTargetTypes(): TargetTypes = LeanTypes

    override fun generateDelayBody(action: Action, port: VarRef): String = TODO()

    override fun generateForwardBody(action: Action?, port: VarRef?): String = TODO()

    override fun generateDelayGeneric(): String = TODO()
}