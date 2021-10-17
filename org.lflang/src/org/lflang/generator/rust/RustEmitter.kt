/*
 * Copyright (c) 2021, TU Dresden.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.lflang.generator.rust

import org.lflang.generator.LocationInfo
import org.lflang.generator.PrependOperator
import org.lflang.generator.TargetCode
import org.lflang.generator.locationInfo
import org.lflang.generator.rust.RustEmitter.generateRustProject
import org.lflang.joinLines
import org.lflang.joinWithCommas
import org.lflang.withDQuotes
import java.nio.file.Paths
import java.time.LocalDateTime
import java.time.format.DateTimeFormatter


/**
 * Part of the Rust generator that emits the actual Rust code,
 * including its project structure. Its entry point is
 * [generateRustProject].
 */
object RustEmitter {
    /** Name of the runtime crate that is in its Cargo.toml.*/
    private const val runtimeCrateFullName = "reactor_rt"

    /** Qualification prefix to refer to a member of the runtime library crate. */
    const val rsRuntime = "::$runtimeCrateFullName"

    fun generateRustProject(fileConfig: RustFileConfig, gen: GenerationInfo) {

        fileConfig.emit("Cargo.toml") { makeCargoTomlFile(gen) }

        // if singleFile, this file will contain every module.
        fileConfig.emit("src/main.rs") { makeMainFile(gen) }

        if (!gen.properties.singleFile) {
            fileConfig.emit("src/reactors/mod.rs") { makeReactorsAggregateModule(gen) }
            for (reactor in gen.reactors) {
                fileConfig.emit("src/reactors/${reactor.names.modFileName}.rs") {
                    makeReactorModule(this, reactor)
                }
            }
        }
    }

    private fun makeReactorModule(out: Emitter, reactor: ReactorInfo) {
        out += with(reactor) {
            val typeParams = typeParamList.map { it.targetCode }.angle()
            val typeArgs = typeParamList.map { it.lfName }.angle()

            with(reactor.names) {
                with(PrependOperator) {
                    """
                |${generatedByComment("//")}
                |#![allow(unused)]
                |
                |use $rsRuntime::prelude::*;
                |
${"             |"..reactor.preambles.joinToString("\n\n") { "// preamble {=\n${it.trimIndent()}\n// =}" }}
                |
                |/// Generated from ${loc.display()}
                |///
                |/${loc.lfTextComment()}
                |pub struct $structName$typeParams {
                |    pub __phantom: std::marker::PhantomData<(${typeParamList.map { it.lfName }.joinWithCommas()})>,
${"             |    "..reactor.stateVars.joinWithCommasLn { it.lfName + ": " + it.type }}
                |}
                |
                |#[warn(unused)]
                |impl$typeParams $structName$typeArgs {
                |
${"             |    "..reactions.joinToString("\n\n") { it.toWorkerFunction() }}
                |
                |}
                |
                |/// Parameters for the construction of a [$structName]
                |pub struct ${names.paramStructName}$typeParams {
                |    pub __phantom: std::marker::PhantomData<(${typeParamList.map { it.lfName }.joinWithCommas()})>,
${"             |    "..ctorParams.joinWithCommasLn { "pub ${it.lfName.escapeRustIdent()}: ${it.type}" }}
                |}
                |
                |
                |//------------------------//
                |
                |
                |pub struct $wrapperName$typeParams {
                |    __id: $rsRuntime::ReactorId,
                |    __impl: $structName$typeArgs,
${"             |    "..otherComponents.joinWithCommasLn { it.toStructField() }}
                |}
                |
                |impl$typeParams $wrapperName$typeArgs {
                |    #[inline]
                |    fn user_assemble(__assembler: &mut $rsRuntime::AssemblyCtx, __params: $paramStructName$typeArgs) -> Self {
                |        let $ctorParamsDeconstructor = __params;
                |        Self {
                |            __id: __assembler.get_id(),
                |            __impl: $structName {
                |                __phantom: std::marker::PhantomData,   
${"             |                "..reactor.stateVars.joinWithCommasLn { it.lfName + ": " + (it.init ?: "Default::default()") }}
                |            },
${"             |            "..otherComponents.joinWithCommasLn { it.rustFieldName + ": " + it.initialExpression() }}
                |        }
                |    }
                |}
                |
                |use $rsRuntime::*; // after this point there's no user-written code
                |
                |impl$typeParams $rsRuntime::ReactorInitializer for $wrapperName$typeArgs {
                |    type Wrapped = $structName$typeArgs;
                |    type Params = $paramStructName$typeArgs;
                |    const MAX_REACTION_ID: LocalReactionId = LocalReactionId::new_const($maxReactionIdUsize);
                |
                |    fn assemble(__params: Self::Params, __assembler: &mut AssemblyCtx) -> Result<Self, AssemblyError> {
                |        // children reactors   
${"             |        "..assembleChildReactors()}
                |
                |        __assembler.fix_cur_id();
                |
                |        // assemble self
                |        let mut __self: Self = Self::user_assemble(__assembler, __params);
                |
${"             |        "..declareReactions()}
                |
                |        {
                |
${"             |            "..graphDependencyDeclarations(reactor)}
                |
${"             |            "..declareChildConnections()}
                |        }
${"             |        "..nestedInstances.joinToString("\n") { "__assembler.register_reactor(${it.rustLocalName});" }}
                |
                |       Ok(__self)
                |    }
                |}
                |
                |
                |impl$typeParams ReactorBehavior for $wrapperName$typeArgs {
                |
                |    #[inline]
                |    fn id(&self) -> ReactorId {
                |        self.__id
                |    }
                |
                |    fn react_erased(&mut self, ctx: &mut ReactionCtx, rid: LocalReactionId) {
                |        match rid.raw() {
${"             |            "..workerFunctionCalls(reactor)}
${"             |            "..reactor.syntheticTimerReactions()}
                |            _ => panic!("Invalid reaction ID: {} should be < {}", rid, Self::MAX_REACTION_ID)
                |        }
                |    }
                |
                |    fn cleanup_tag(&mut self, ctx: &CleanupCtx) {
${"             |        "..reactor.otherComponents.mapNotNull { it.cleanupAction() }.joinLn()}
                |    }
                |
                |}
        """.trimMargin()
                }
            }
        }
    }

    /** timers have a reschedule and a bootstrap reaction*/
    private val ReactorInfo.maxReactionIdUsize get() = reactions.size + 2 * timers.size

    private fun ReactorInfo.assembleChildReactors(): String {
        fun NestedReactorInstance.paramStruct(): String =
            args.entries.joinWithCommas("super::${names.paramStructName} {  __phantom: std::marker::PhantomData, ", " }") {
                if (it.key == it.value) it.key.escapeRustIdent()
                else it.key.escapeRustIdent() + ": " + it.value // do not escape value
            }

        val asTuple = nestedInstances.joinWithCommas("($ctorParamsDeconstructor, ", ")") { it.rustLocalName }
        val asMutTuple = nestedInstances.joinWithCommas("(__params, ", ")") { "mut ${it.rustLocalName}" }

        val declarations = nestedInstances.joinToString("\n") {
            """
                ${it.loc.lfTextComment()}
                let ${it.rustLocalName}: super::${it.names.wrapperName}${it.typeArgs.angle()} = __assembler.assemble_sub("${it.lfName}", ${it.paramStruct()})?;
            """.trimIndent()
        }

        // we do this to only bring the arguments in scope
        // within the block
        return with(PrependOperator) { """
            |let $asMutTuple = {
            |    let $ctorParamsDeconstructor = __params;
${"         |    "..declarations}
            |    $asTuple
            |};
        """.trimMargin()
        }
    }


    private fun ReactorInfo.declareChildConnections(): String {
        return connections.joinToString("\n", prefix = "// Declare connections\n") {
            it.locationInfo().lfTextComment() + "\n" +
                    PortEmitter.declareConnection(it)
        } + "\n" + portReferences.joinToString("\n", prefix = "// Declare port references\n") {
            PortEmitter.declarePortRef(it)
        }
    }

    private fun ReactorInfo.declareReactions(): String {
        val reactionIds = reactions.map { it.invokerId } +
                timers.map { it.rescheduleReactionId } +
                timers.map { it.startReactionId }

        val debugLabels = reactions.map { it.debugLabel.toRustOption() } +
                timers.map { "Some(\"reschedule_${it.lfName}\")" } +
                timers.map { "Some(\"bootstrap_${it.lfName}\")" }


        val pattern = reactionIds.joinToString(prefix = "let [", separator = ",\n     ", postfix = "]")
        val debugLabelArray = debugLabels.joinToString(", ", "[", "]")

        return "$pattern = __assembler.new_reactions(${reactions.size}, $debugLabelArray);"
    }

    private fun workerFunctionCalls(reactor: ReactorInfo): String {

        fun joinDependencies(reaction: ReactionInfo): String = sequence {
            for ((kind, deps) in reaction.allDependencies) {
                for (comp in deps) {
                    if (comp.isNotInjectedInReaction(kind, reaction)) continue

                    val borrow = comp.toBorrow(kind) ?: continue
                    yield(borrow)
                }
            }
        }.toList().let {
            if (it.isEmpty()) ""
            else it.joinWithCommas(prefix = ", ")
        }

        return reactor.reactions.joinWithCommasLn(trailing = true) { n: ReactionInfo ->
            "${n.idx} => self.__impl.${n.workerId}(ctx${joinDependencies(n)})"
        }
    }

    private fun ReactorInfo.syntheticTimerReactions(): String {
        val branches = timers.map {
            "${timerReactionId(it, 0)} => ctx.reschedule_timer(&mut self.${it.rustFieldName})"
        } + timers.map {
            "${timerReactionId(it, 1)} => ctx.bootstrap_timer(&mut self.${it.rustFieldName})"
        }
        return branches.joinWithCommasLn(trailing = true)
    }

    private fun ReactorInfo.timerReactionId(timer: TimerData, synthesisNum: Int) =
        reactions.size +
                timers.indexOf(timer).also { assert(it != -1) } +
                synthesisNum * timers.size // offset it by a block

    private fun graphDependencyDeclarations(reactor: ReactorInfo): String {
        val reactions = reactor.reactions.map { n ->
            val deps: List<String> = mutableListOf<String>().apply {
                this += n.triggers.map { trigger -> "__assembler.declare_triggers(__self.${trigger.rustFieldName}.get_id(), ${n.invokerId})?;" }
                if (n.isStartup)
                    this += "__assembler.declare_triggers($rsRuntime::TriggerId::STARTUP, ${n.invokerId})?;"
                if (n.isShutdown)
                    this += "__assembler.declare_triggers($rsRuntime::TriggerId::SHUTDOWN, ${n.invokerId})?;"
                this += n.uses.map { trigger -> "__assembler.declare_uses(${n.invokerId}, __self.${trigger.rustFieldName}.get_id())?;" }
                this += n.effects.filterIsInstance<PortData>()
                    .map { port -> "__assembler.effects_port(${n.invokerId}, &__self.${port.rustFieldName})?;" }
            }

            n.loc.lfTextComment() + "\n" + deps.joinLn()
        }.joinLn()

        val timers = reactor.timers.flatMap {
            listOf(
                "__assembler.declare_triggers(__self.${it.rustFieldName}.get_id(), ${it.rescheduleReactionId})?;",
                // start reactions may "trigger" the timer, otherwise it schedules it
                "__assembler.declare_triggers($rsRuntime::TriggerId::STARTUP, ${it.startReactionId})?;",
                "__assembler.effects_instantaneous(${it.startReactionId}, __self.${it.rustFieldName}.get_id())?;",
            )
        }.joinLn()

        return (reactions + "\n\n" + timers).trimEnd()
    }

    private val TimerData.rescheduleReactionId: String
        get() = "_timer_schedule_$lfName"

    private val TimerData.startReactionId: String
        get() = "_timer_start_$lfName"

    private fun Emitter.makeMainFile(gen: GenerationInfo) {
        val mainReactor = gen.mainReactor
        val mainReactorNames = mainReactor.names
        this += with(PrependOperator) {
            """
            |${generatedByComment("//")}
            |#![allow(unused_imports)]
            |#![allow(non_snake_case)]
            |
            |#[macro_use]
            |extern crate $runtimeCrateFullName;
            |#[macro_use]
            |extern crate assert_matches;
            |extern crate env_logger;
            |
            |use $rsRuntime::*;
            |use self::reactors::${mainReactorNames.wrapperName} as _MainReactor;
            |use self::reactors::${mainReactorNames.paramStructName} as _MainParams;
            |
            |fn main() {
            |    init_logger();
            |
            |    // todo CLI parsing
            |    let options = SchedulerOptions {
            |       timeout: ${gen.properties.timeout.toRustOption()},
            |       keep_alive: ${gen.properties.keepAlive}
            |    };
            |    // todo main params are entirely defaulted for now.
            |    let main_args = _MainParams {
            |       __phantom: std::marker::PhantomData,
${"         |       "..mainReactor.ctorParams.joinWithCommasLn { it.lfName.escapeRustIdent() + ":" + (it.defaultValue ?: "Default::default()") }}
            |    };
            |
            |    SyncScheduler::run_main::<_MainReactor>(options, main_args);
            |}
            |
            |fn init_logger() {
            |    env_logger::Builder::from_env(env_logger::Env::default())
            |        .format_target(false)
            |        .init();
            |}
            |
        """.trimMargin()
        }

        skipLines(2)

        if (gen.properties.singleFile) {
            makeSingleFileProject(gen)
        } else {
            this += "mod reactors;\n"
        }
    }

    private fun Emitter.makeSingleFileProject(gen: GenerationInfo) {
        this += """
            |//-------------------//
            |//---- REACTORS -----//
            |//-------------------//
            |
        """.trimMargin()

        this.writeInBlock("mod reactors {") {
            for (reactor in gen.reactors) {
                this += with(reactor.names) {
                    """
                pub use self::$modName::$wrapperName;
                pub use self::$modName::$paramStructName;
                """.trimIndent()
                }
                skipLines(1)
            }

            for (reactor in gen.reactors) {
                this += """
                    |//--------------------------------------------//
                    |//------------ ${reactor.lfName} -------//
                    |//-------------------//
                    """.trimMargin()

                this.writeInBlock("mod ${reactor.names.modName} {") {
                    makeReactorModule(this, reactor)
                }
                this.skipLines(2)
            }
        }
    }


    private fun Emitter.makeReactorsAggregateModule(gen: GenerationInfo) {
        fun ReactorInfo.modDecl(): String = with(names) {
            // We make some declarations public to be able to refer to them
            // simply when building nested reactors.
            """
                mod $modName;
                pub use self::$modName::$wrapperName;
                pub use self::$modName::$paramStructName;
            """.trimIndent()
        }

        this += with(PrependOperator) {
            """
            |${generatedByComment("//")}
            |
${"         |"..gen.reactors.joinToString("\n") { it.modDecl() }}
            |
        """.trimMargin()
        }
    }

    private fun Emitter.makeCargoTomlFile(gen: GenerationInfo) {
        val (crate) = gen
        this += """
            |${generatedByComment("#")}
            |[package]
            |name = "${crate.name}"
            |version = "${crate.version}"
            |authors = [${crate.authors.joinToString(", ") { it.withDQuotes() }}]
            |edition = "2018"
            |
            |[dependencies]
            |env_logger = "0.9"
            |assert_matches = "1.5.0" # useful for tests
            |
            |[dependencies.$runtimeCrateFullName] # the reactor runtime
            |${gen.runtime.runtimeCrateSpec()}
            |
            |[[bin]]
            |name = "${gen.executableName}"
            |path = "src/main.rs"
            |
        """.trimMargin()
    }


    private fun RuntimeInfo.runtimeCrateSpec(): String =
        buildString {
            if (localPath != null) {
                val localPath = Paths.get(localPath).toAbsolutePath().toString().escapeStringLiteral()

                appendLine("path = \"$localPath\"")
            } else {
                appendLine("git = \"https://github.com/lf-lang/reactor-rust.git\"")
                if (version != null) {
                    // Version just checks out a relevant tag, while we're not published to crates.io
                    // Maybe we'll never need to be published.
                    appendLine("tag=\"v$version\"")
                } else {
                    appendLine("rev=\"$gitRevision\"")
                }
            }
        }

    /// Rust pattern that deconstructs a ctor param tuple into individual variables
    private val ReactorInfo.ctorParamsDeconstructor: TargetCode
        get() {
            val fields = ctorParams.joinWithCommas { it.lfName.escapeRustIdent() }
            return "${names.paramStructName} {  __phantom, $fields }"
        }


    /**
     * Returns null if there is no need to manipulate the
     * dependency within the reaction.
     */
    private fun ReactorComponent.toBorrow(kind: DepKind): TargetCode? = when (this) {
        is PortData, is ChildPortReference ->
            if (kind == DepKind.Effects) "$rsRuntime::WritablePort::new(&mut self.$rustFieldName)"
            else "&$rsRuntime::ReadablePort::new(&self.$rustFieldName)"
        is ActionData                      -> if (kind == DepKind.Effects) "&mut self.$rustFieldName" else "&self.$rustFieldName"
        is TimerData                       -> "&self.$rustFieldName"
    }

    private fun ReactorComponent.isNotInjectedInReaction(depKind: DepKind, n: ReactionInfo): Boolean =
                // Item is both in inputs and outputs.
                // We must not generate 2 parameters, instead we generate the
                // one with the most permissions (Effects means &mut).

                // eg `reaction(act) -> act` must not generate 2 parameters for act,
                // we skip the Trigger one and generate the Effects one.
                depKind != DepKind.Effects && this in n.effects

    private fun ReactorComponent.isInjectedAsMut(depKind: DepKind): Boolean =
        depKind == DepKind.Effects && (this is PortData || this is ActionData)

    /**
     * Whether this component may be unused in a reaction.
     * Eg. actions on which we have just a trigger dependency
     * are fine to ignore.
     */
    private fun ReactorComponent.mayBeUnusedInReaction(depKind: DepKind): Boolean =
        depKind == DepKind.Triggers && this !is PortData

    private fun ReactorComponent.toBorrowedType(kind: DepKind): TargetCode =
        when (this) {
            is PortData           -> portRefWrapper(kind, dataType)
            is ChildPortReference -> portRefWrapper(kind, dataType)
            is TimerData          -> "&${toType()}"
            is ActionData          -> if (kind == DepKind.Effects) "&mut ${toType()}" else "&${toType()}"
        }

    private fun portRefWrapper(kind: DepKind, dataType: TargetCode) =
        if (kind == DepKind.Effects) "$rsRuntime::WritablePort<$dataType>" // note: owned
        else "&$rsRuntime::ReadablePort<$dataType>" // note: a reference

    private fun ReactorComponent.toType(): TargetCode = when (this) {
        is ActionData         ->
            if (isLogical) "$rsRuntime::LogicalAction<${dataType ?: "()"}>"
            else "$rsRuntime::PhysicalActionRef<${dataType ?: "()"}>"
        is PortData           -> "$rsRuntime::Port<$dataType>"
        is ChildPortReference -> "$rsRuntime::Port<$dataType>"
        is TimerData          -> "$rsRuntime::Timer"
    }

    private fun ReactorComponent.initialExpression(): TargetCode = when (this) {
        is ActionData         -> {
            val delay = minDelay.toRustOption()
            val ctorName = if (isLogical) "new_logical_action" else "new_physical_action"
            "__assembler.$ctorName(\"$lfName\", $delay)"
        }
        is TimerData          -> "__assembler.new_timer(\"$lfName\", $offset, $period)"
        is PortData           -> "__assembler.new_port(\"$lfName\", $isInput)"
        is ChildPortReference -> "__assembler.new_port(\"$childName.$lfName\", $isInput)"
    }

    private fun ReactorComponent.cleanupAction(): TargetCode? = when (this) {
        is ActionData                      ->
            if (isLogical) "ctx.cleanup_logical_action(&mut self.$rustFieldName);"
            else "ctx.cleanup_physical_action(&mut self.$rustFieldName);"
        is PortData, is ChildPortReference -> "ctx.cleanup_port(&mut self.$rustFieldName);"
        is TimerData                       -> null
    }


    private fun ReactorComponent.toStructField(): TargetCode {
        val fieldVisibility = if (this is PortData) "pub " else ""
        return "$fieldVisibility$rustFieldName: ${toType()}"
    }

    private fun ReactionInfo.toWorkerFunction(): String {
        fun ReactionInfo.reactionParams(): List<String> = sequence {
            for ((kind, comps) in allDependencies) {
                for (comp in comps) {
                    if (comp.isNotInjectedInReaction(kind, this@reactionParams)) continue

                    // we want the user to be able to make
                    // use of the mut if they want, but they
                    // don't have to
                    val mut = if (comp.isInjectedAsMut(kind)) "#[allow(unused_mut)] mut " else ""

                    val param = "$mut${comp.rustRefName}: ${comp.toBorrowedType(kind)}"

                    if (comp.mayBeUnusedInReaction(kind)) {
                        yield("#[allow(unused)] $param")
                    } else {
                        yield(param)
                    }
                }
            }
        }.toList()

        val indent = " ".repeat("fn $workerId(".length)
        return with(PrependOperator) {
            """
                |${loc.lfTextComment()}
                |fn $workerId(&mut self, 
                |$indent#[allow(unused)] ctx: &mut $rsRuntime::ReactionCtx,
${"             |$indent"..reactionParams().joinWithCommasLn { it }}) {
${"             |    "..body}
                |}
            """.trimMargin()
        }
    }



}

/**
 * Produce a commented out version of the text of this AST node.
 * This is helpful to figure out how the rust code corresponds to
 * the LF code.
 */
private fun LocationInfo.lfTextComment() =
    "// --- ${lfText.joinLines()}"

private val timeFormatter = DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss")
private fun generatedByComment(delim: String) =
    "$delim-- Generated by LFC @ ${timeFormatter.format(LocalDateTime.now())} --$delim"

private fun TargetCode?.toRustOption(): TargetCode =
    if (this == null) "None"
    else "Some($this)"

private fun Iterable<CharSequence>.joinLn(): String =
    joinToString("\n")

private fun <T> Iterable<T>.joinWithCommas(
    prefix: CharSequence = "",
    postfix: CharSequence = "",
    skipLines: Boolean = false,
    trailing: Boolean = true,
    transform: (T) -> CharSequence = { it.toString() }
): String {
    val delim =
        (if (skipLines) "\n" else " ")
            .let { if (trailing) it else ",$it" }

    return joinToString(delim, prefix, postfix) { t ->
        transform(t).let { if (trailing) "$it," else it }
    }
}

private fun <T> List<T>.toVecLiteral(transform: (T) -> String = { it.toString() }) =
    joinWithCommas("vec![", "]", transform = transform)

private fun <T> Iterable<T>.joinWithCommasLn(
    prefix: CharSequence = "",
    postfix: CharSequence = "",
    trailing: Boolean = true,
    transform: (T) -> CharSequence = { it.toString() }
): String = joinWithCommas(prefix, postfix, skipLines = true, trailing, transform)

fun List<TargetCode>.angle() = if (this.isEmpty()) "" else joinWithCommas("<", ">")

/**
 * If this is a rust keyword, escape it for it to be interpreted
 * as an identifier, except if this keyword is a valid rust expression.
 */
fun String.escapeRustIdent() = RustTypes.escapeIdentifier(this)


fun String.escapeStringLiteral() =
    replace(Regex("[\\\\ \t]")) {
        when (it.value) {
            "\\" -> "\\\\"
            "\t" -> "\\t"
            "\"" -> "\\\""
            else -> it.value
        }
    }
