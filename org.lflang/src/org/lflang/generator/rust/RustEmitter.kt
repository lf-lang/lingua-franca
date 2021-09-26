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

import org.lflang.Target
import org.lflang.generator.LocationInfo
import org.lflang.generator.PrependOperator
import org.lflang.generator.TargetCode
import org.lflang.generator.locationInfo
import org.lflang.generator.rust.RustEmitter.generateRustProject
import org.lflang.generator.rust.RustEmitter.rsRuntime
import org.lflang.joinLines
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
            val typeArgs = typeParamList.map { it.rustName.escapeRustIdent() }.angle()

            with (reactor.names) {
            with(ReactorComponentEmitter) {
                with(PrependOperator) {
                    """
                |${generatedByComment("//")}
                |#![allow(unused)]
                |
                |use $rsRuntime::{LogicalInstant, PhysicalInstant, Duration};
                |use $rsRuntime::Offset::{After, Asap};
                |use std::sync::{Arc, Mutex};
                |
${"             |"..reactor.preambles.joinToString("\n\n") { "// preamble {=\n${it.trimIndent()}\n// =}" }}
                |
                |/// Generated from ${loc.display()}
                |///
                |/${loc.lfTextComment()}
                |pub struct $structName$typeParams {
${"             |    "..reactor.stateVars.joinWithCommasLn { it.lfName + ": " + it.type }}
                |}
                |
                |#[warn(unused)]
                |impl$typeParams $structName$typeArgs {
                |
${"             |    "..reactions.joinToString("\n\n") { it.toWorkerFunction(reactor) }}
                |
                |}
                |
                |/// Parameters for the construction of a [$structName]
                |#[derive(Clone)]
                |pub struct ${names.paramStructName} {
${"             |    "..ctorParams.joinWithCommasLn { "pub ${it.lfName}: ${it.type}" }}
                |}
                |
                |
                |//------------------------//
                |
                |
                |pub struct $wrapperName$typeParams {
                |    _id: $rsRuntime::ReactorId,
                |    _impl: $structName$typeArgs,
                |    _params: $paramStructName,
                |    _startup_reactions: $rsRuntime::ReactionSet,
                |    _shutdown_reactions: $rsRuntime::ReactionSet,
${"             |    "..(otherComponents + portReferences).joinWithCommasLn { it.toStructField() }}
                |}
                |
                |impl$typeParams $wrapperName$typeArgs {
                |    #[inline]
                |    fn user_assemble(_assembler: &mut $rsRuntime::AssemblyCtx, _params: $paramStructName) -> Self {
                |        let $ctorParamsDeconstructor = _params.clone();
                |        Self {
                |            _id: _assembler.get_id(),
                |            _params,
                |            _startup_reactions: Default::default(),
                |            _shutdown_reactions: Default::default(),
                |            _impl: $structName {
${"             |                "..reactor.stateVars.joinWithCommasLn { it.lfName + ": " + (it.init ?: "Default::default()") }}
                |            },
${"             |            "..(otherComponents + portReferences).joinWithCommasLn { it.rustFieldName + ": " + it.initialExpression() }}
                |        }
                |    }
                |}
                |
                |use $rsRuntime::*; // after this point there's no user-written code
                |
                |impl$typeParams $rsRuntime::ReactorInitializer for $wrapperName$typeArgs {
                |    type Wrapped = $structName$typeArgs;
                |    type Params = $paramStructName;
                |    const MAX_REACTION_ID: LocalReactionId = LocalReactionId::new_const(${reactions.size + timers.size /*timers have a reschedule reaction*/});
                |
                |    fn assemble(args: Self::Params, _assembler: &mut AssemblyCtx) -> Result<Self, AssemblyError> {
                |        // children reactors   
${"             |        "..assembleChildReactors()}
                |
                |        _assembler.fix_cur_id();
                |
                |        // assemble self
                |        let mut _self: Self = Self::user_assemble(_assembler, args);
                |
${"             |        "..declareReactions()}
                |
                |        {
                |            _self._startup_reactions = ${reactions.filter { it.isStartup }.toVecLiteral { it.invokerId }};
                |            _self._shutdown_reactions = ${reactions.filter { it.isShutdown }.toVecLiteral { it.invokerId }};
                |
${"             |            "..graphDependencyDeclarations(reactor)}
                |
${"             |            "..declareChildConnections()}
                |        }
${"             |        "..nestedInstances.joinToString("\n") { "_assembler.register_reactor(${it.rustLocalName});" }}
                |
                |       Ok(_self)
                |    }
                |}
                |
                |
                |impl$typeParams ReactorBehavior for $wrapperName$typeArgs {
                |
                |    #[inline]
                |    fn id(&self) -> ReactorId {
                |        self._id
                |    }
                |
                |    fn react_erased(&mut self, ctx: &mut ReactionCtx, rid: LocalReactionId) {
                |        match rid.raw() {
${"             |            "..workerFunctionCalls(reactor)}
${"             |            "..syntheticTimerReactions(reactor)}
                |            _ => panic!("Invalid reaction ID: {} should be < {}", rid, Self::MAX_REACTION_ID)
                |        }
                |    }
                |
                |    fn cleanup_tag(&mut self, ctx: &CleanupCtx) {
${"             |        "..reactor.otherComponents.mapNotNull { it.cleanupAction() }.joinLn() }
                |    }
                |    
                |    fn enqueue_startup(&self, ctx: &mut StartupCtx) {
                |        ctx.enqueue(&self._startup_reactions);
${"             |        "..reactor.timers.joinToString("\n") { "ctx.start_timer(&self.${it.rustFieldName});" }}
                |    }
                |
                |    fn enqueue_shutdown(&self, ctx: &mut StartupCtx) {
                |        ctx.enqueue(&self._shutdown_reactions);
                |    }
                |
                |}
        """.trimMargin()
                }
            }
            }
        }
    }

    private fun ReactorInfo.assembleChildReactors(): String {
        fun NestedReactorInstance.paramStruct(): String =
            args.entries.joinWithCommas("super::${names.paramStructName} { ", " }") {
                if (it.key == it.value) it.key.escapeRustIdent()
                else it.key.escapeRustIdent() + ": " + it.value.escapeRustIdent()
            }

        val asTuple = nestedInstances.joinWithCommas("(", ")") { it.rustLocalName }
        val asMutTuple = nestedInstances.joinWithCommas("(", ")") { "mut ${it.rustLocalName}" }

        val declarations = nestedInstances.joinToString("\n") {
            """
                ${it.loc.lfTextComment()}
                let ${it.rustLocalName}: super::${it.names.wrapperName}${it.typeArgs.angle()} = _assembler.assemble_sub("${it.lfName}", ${it.paramStruct()})?;
            """.trimIndent()
        }

        // we do this to only bring the arguments in scope
        // within the block
        return with(PrependOperator) { """
            |let $asMutTuple = {
            |    let $ctorParamsDeconstructor = args.clone();
${"         |    "..declarations}
            |    $asTuple
            |};
        """.trimMargin()
        }
    }


    private fun ReactorInfo.declareChildConnections(): String {
        return connections.joinToString("\n", "// Declare connections\n") {
            it.locationInfo().lfTextComment() + "\n" +
                    PortEmitter.declareConnection(it)
        } + portReferences.joinToString("// Declare port references") { // fixme add \n and a test
            PortEmitter.declarePortRef(it)
        }
    }

    private fun ReactorInfo.declareReactions(): String {
        val reactionIds = reactions.map { it.invokerId } + timers.map { it.rescheduleReactionId }

        val pattern = reactionIds.joinToString(prefix = "let [", separator = ",\n     ", postfix = "]")

        return "$pattern = _assembler.new_reactions::<{Self::MAX_REACTION_ID.index()}>();"
    }

    private fun ReactorComponentEmitter.workerFunctionCalls(reactor: ReactorInfo): String {

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
            "${n.idx} => self._impl.${n.workerId}(ctx, &self._params${joinDependencies(n)})"
        }
    }

    private fun syntheticTimerReactions(reactor: ReactorInfo): String {
        return reactor.timers.joinWithCommasLn(trailing = true) { timer: TimerData ->
            "${reactor.timerReactionId(timer)} => ctx.maybe_reschedule(&self.${timer.rustFieldName})"
        }
    }

    private fun ReactorInfo.timerReactionId(timer: TimerData) =
        reactions.size + timers.indexOf(timer).also { assert(it != -1) }

    private fun graphDependencyDeclarations(reactor: ReactorInfo): String {
        val reactions = reactor.reactions.map { n ->
            val deps =
                n.triggers.map { trigger -> "_assembler.declare_triggers(_self.${trigger.rustFieldName}.get_id(), ${n.invokerId})?;" } +
                        n.effects.filterIsInstance<PortData>()
                            .map { port -> "_assembler.effects_port(${n.invokerId}, &_self.${port.rustFieldName})?;" } +
                        n.uses.map { trigger -> "_assembler.declare_uses(${n.invokerId}, _self.${trigger.rustFieldName}.get_id())?;" }

            n.loc.lfTextComment() + "\n" + deps.joinLn()
        }.joinLn()

        val timers = reactor.timers.map { "_assembler.declare_triggers(_self.${it.rustFieldName}.get_id(), ${it.rescheduleReactionId})?;" }.joinLn()

        return (reactions + "\n\n" + timers).trimEnd()
    }

    private val TimerData.rescheduleReactionId: String
        get() = "_timer_schedule_$lfName"

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
${"         |       "..mainReactor.ctorParams.joinWithCommasLn { it.lfName + ":" + (it.defaultValue ?: "Default::default()") }}
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
            |# The reactor runtime
            |$runtimeCrateFullName = { ${gen.runtime.runtimeCrateSpec()} }
            |# Other dependencies
            |env_logger = "0.9"
            |assert_matches = "1.5.0" # useful for tests
            |
            |[[bin]]
            |name = "${gen.executableName}"
            |path = "src/main.rs"
        """.trimMargin()
    }


    private fun RuntimeInfo.runtimeCrateSpec(): String =
        buildString {
            if (version != null) append("version=\"$version\" ")

            if (localPath != null) {
                append("path = \"${Paths.get(localPath).toAbsolutePath()}\"")
            } else {
                append("git = \"ssh://git@github.com/lf-lang/reactor-rust.git\"")
            }
        }

    /// Rust pattern that deconstructs a ctor param tuple into individual variables
    private val ReactorInfo.ctorParamsDeconstructor: TargetCode
        get() {
            val fields = ctorParams.joinWithCommas { it.lfName.escapeRustIdent() }
            return "${names.paramStructName} { $fields }"
        }
}


private object ReactorComponentEmitter {


    /**
     * Returns null if there is no need to manipulate the
     * dependency within the reaction.
     */
    fun ReactorComponent.toBorrow(kind: DepKind): TargetCode? = when (this) {
        is PortData, is ChildPortReference ->
            if (kind == DepKind.Effects) "$rsRuntime::WritablePort::new(&mut self.$rustFieldName)"
            else "$rsRuntime::ReadablePort::new(&self.$rustFieldName)"
        is ActionData                      -> "&mut self.$rustFieldName"
        is TimerData                       -> null
    }

    fun ReactorComponent.isNotInjectedInReaction(depKind: DepKind, n: ReactionInfo): Boolean =
        this is TimerData
                // Item is both in inputs and outputs.
                // We must not generate 2 parameters, instead we generate the
                // one with the most permissions (Effects means &mut).

                // eg `reaction(act) -> act` must not generate 2 parameters for act,
                // we skip the Trigger one and generate the Effects one.
                || depKind != DepKind.Effects && this in n.effects

    fun ReactorComponent.isInjectedAsMut(depKind: DepKind): Boolean =
        depKind == DepKind.Effects && (this is PortData || this is ActionData)

    /**
     * Whether this component may be unused in a reaction.
     * Eg. actions on which we have just a trigger dependency
     * are fine to ignore.
     */
    fun ReactorComponent.mayBeUnusedInReaction(depKind: DepKind): Boolean =
        depKind == DepKind.Triggers && this !is PortData

    fun ReactorComponent.toBorrowedType(kind: DepKind): TargetCode =
        if (this is PortData || this is ChildPortReference) {
            val dataType = (this as DataTypeOwner).dataType

            if (kind == DepKind.Effects) "$rsRuntime::WritablePort<$dataType>"
            else "$rsRuntime::ReadablePort<$dataType>"
        } else "&mut ${toType()}"

    fun ReactorComponent.toType(): TargetCode = when (this) {
        is ActionData         ->
            if (isLogical) "$rsRuntime::LogicalAction<${dataType ?: "()"}>"
            else "$rsRuntime::PhysicalAction<${dataType ?: "()"}>"
        is PortData           -> "$rsRuntime::Port<$dataType>"
        is ChildPortReference -> "$rsRuntime::Port<$dataType>"
        is TimerData          -> "$rsRuntime::Timer"
    }

    fun ReactorComponent.initialExpression(): TargetCode = when (this) {
        is ActionData         -> {
            val delay = minDelay.toRustOption()
            val ctorName = if (isLogical) "new_logical_action" else "new_physical_action"
            "_assembler.$ctorName(\"$lfName\", $delay)"
        }
        is TimerData          -> "_assembler.new_timer(\"$lfName\", $offset, $period)"
        is PortData           -> "_assembler.new_port(\"$lfName\")"
        is ChildPortReference -> "_assembler.new_port(\"$childName.$lfName\")"
    }

    fun ReactorComponent.cleanupAction(): TargetCode? = when (this) {
        is ActionData         -> "ctx.cleanup_action(&mut self.$rustFieldName);"
        is PortData,
        is ChildPortReference -> "ctx.cleanup_port(&mut self.$rustFieldName);"
        is TimerData          -> null
    }


    fun ReactorComponent.toStructField(): TargetCode {
        val fieldVisibility = if (this is PortData) "pub " else ""
        return "$fieldVisibility$rustFieldName: ${toType()}"
    }

    fun ReactionInfo.toWorkerFunction(reactor: ReactorInfo): String {
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
                |$indent#[allow(unused)] params: &${reactor.names.paramStructName},
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

fun String.escapeRustIdent() =
    if (this in Target.Rust.keywords) "r#$this" else this
