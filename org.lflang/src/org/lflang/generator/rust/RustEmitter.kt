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

import org.lflang.*
import org.lflang.generator.PrependOperator
import org.lflang.generator.PrependOperator.rangeTo
import org.lflang.generator.TargetCode
import org.lflang.generator.UnsupportedGeneratorFeatureException
import org.lflang.generator.rust.RustEmitter.generateRustProject
import java.nio.file.Files


/**
 * Part of the Rust generator that emits the actual Rust code,
 * including its project structure. Its entry point is
 * [generateRustProject].
 */
object RustEmitter : RustEmitterBase() {

    fun generateRustProject(fileConfig: RustFileConfig, gen: GenerationInfo) {

        fileConfig.emit("Cargo.toml") {
            with(RustCargoTomlEmitter) {
                makeCargoTomlFile(gen)
            }
        }

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

        // copy user-defined modules
        for (modPath in gen.crate.modulesToIncludeInMain) {
            val target = fileConfig.srcGenPath.resolve("src").resolve(modPath.fileName)
            if (Files.isDirectory(modPath)) {
                FileConfig.copyDirectory(modPath, target)
            } else {
                FileConfig.copyFile(modPath, target)
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
                |    __phantom: std::marker::PhantomData<(${typeParamList.map { it.lfName }.joinWithCommas()})>,
${"             |    "..ctorParams.joinWithCommasLn { "${it.lfName.escapeRustIdent()}: ${it.type}" }}
                |}
                |
                |impl$typeParams ${names.paramStructName}$typeArgs {
                |   pub fn new(
${"             |       "..ctorParams.joinWithCommasLn { "${it.lfName.escapeRustIdent()}: ${it.type}" }}
                |   ) -> Self {
                |       Self { __phantom: std::marker::PhantomData, ${ctorParams.joinWithCommas { it.lfName.escapeRustIdent() }} }
                |   }
                |}
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
                |    fn user_assemble(__assembler: &mut $rsRuntime::assembly::ComponentCreator<Self>,
                |                     __id: $rsRuntime::ReactorId,
                |                     __params: $paramStructName$typeArgs) -> $rsRuntime::assembly::AssemblyResult<Self> {
                |        let $ctorParamsDeconstructor = __params;
                |
                |        let __impl = {
                |            // declare them all here so that they are visible to the initializers of state vars declared later
${"             |            "..reactor.stateVars.joinToString("\n") { "let ${it.lfName} = ${it.init};" }}
                |
                |            $structName {
                |                __phantom: std::marker::PhantomData,
${"             |                "..reactor.stateVars.joinWithCommasLn { it.lfName }}
                |            }
                |        };
                |
                |        Ok(Self {
                |            __id,
                |            __impl,
${"             |            "..otherComponents.joinWithCommasLn { it.rustFieldName + ": " + it.initialExpression() }}
                |        })
                |    }
                |}
                |
                |impl$typeParams $rsRuntime::assembly::ReactorInitializer for $wrapperName$typeArgs {
                |    type Wrapped = $structName$typeArgs;
                |    type Params = $paramStructName$typeArgs;
                |    const MAX_REACTION_ID: $rsRuntime::LocalReactionId = $rsRuntime::LocalReactionId::new($totalNumReactions - 1);
                |
                |    fn assemble(__params: Self::Params, __ctx: $rsRuntime::assembly::AssemblyCtx<Self>) -> $rsRuntime::assembly::AssemblyResult<Self> {
                |        use $rsRuntime::assembly::TriggerLike;
                |
                |        let $ctorParamsDeconstructor = __params;
                |        let (_, __self) =
${"             |            "..assembleChildReactors(assembleSelfCall())};
                |
                |       Ok(__self)
                |    }
                |}
                |
                |
                |impl$typeParams $rsRuntime::ReactorBehavior for $wrapperName$typeArgs {
                |
                |    #[inline]
                |    fn id(&self) -> $rsRuntime::ReactorId {
                |        self.__id
                |    }
                |
                |    fn react_erased(&mut self, ctx: &mut $rsRuntime::ReactionCtx, rid: $rsRuntime::LocalReactionId) {
                |        match rid.raw() {
${"             |            "..workerFunctionCalls()}
${"             |            "..syntheticTimerReactions()}
                |            _ => panic!("Invalid reaction ID: {} should be < {}", rid, <Self as $rsRuntime::assembly::ReactorInitializer>::MAX_REACTION_ID)
                |        }
                |    }
                |
                |    fn cleanup_tag(&mut self, ctx: &$rsRuntime::CleanupCtx) {
${"             |        "..otherComponents.mapNotNull { it.cleanupAction() }.joinLn()}
                |    }
                |
                |}
        """.trimMargin()
                }
            }
        }
    }

    private fun ReactorInfo.assembleChildReactors(assembleSelf: String): String {
        fun NestedReactorInstance.childDeclaration(): String {
            val type = "super::${names.wrapperName}${typeArgs.angle()}"
            val params = args.values.joinWithCommas("super::${names.paramStructName}::new(", ")")

            return if (bankWidth != null)
                "__ctx.with_child_bank::<$type, _, _>(\"$lfName\", ${bankWidth.toRustExpr()}, |bank_index| $params, |mut __ctx, $rustLocalName| {"
            else
                "__ctx.with_child::<$type, _>(\"$lfName\", $params, |mut __ctx, $rustLocalName| {"
        }

        return buildString {
            for (inst in nestedInstances) {
                append(inst.childDeclaration()).append("\n")
            }

            append(assembleSelf).append("\n")

            for (inst in nestedInstances) {
                append("})")
            }
            append("?")
        }
    }

    private fun ReactorInfo.assembleSelfCall(): String {
        val reactionIds = reactions.map { it.invokerId } +
                timers.map { it.rescheduleReactionId } +
                timers.map { it.startReactionId }

        val debugLabels = reactions.map { it.debugLabel?.withDQuotes().toRustOption() } +
                timers.map { "Some(\"reschedule_${it.lfName}\")" } +
                timers.map { "Some(\"bootstrap_${it.lfName}\")" }


        val pattern = reactionIds.joinToString(prefix = "[", separator = ", ", postfix = "]")
        val debugLabelArray = debugLabels.joinToString(", ", "[", "]")

        return """
                |__ctx.do_assembly(
                |    |cc, id| Self::user_assemble(cc, id, $ctorParamsDeconstructor),
                |    // number of non-synthetic reactions
                |    ${reactions.size},
                |    // reaction debug labels
                |    $debugLabelArray,
                |    // dependency declarations
                |    |__assembler, __self, $pattern| {
                |        #[allow(unused)]
                |        use reactor_rt::unsafe_iter_bank;
${"             |        "..graphDependencyDeclarations()}
${"             |        "..declareChildConnections()}
                |
                |        Ok(())
                |    }
                |)
            """.trimMargin()
    }


    private fun ReactorInfo.declareChildConnections(): String = with(PortEmitter) {
        connections.joinToString("\n", prefix = "// Declare connections\n") {
            it.declareConnection("__assembler")
        } + "\n" + portReferences.joinToString("\n", prefix = "// Declare port references\n") {
            it.declarePortRef("__assembler")
        }
    }

    /** Renders calls to worker functions from within the react_erased function. */
    private fun ReactorInfo.workerFunctionCalls(): TargetCode {

        fun joinDependencies(reaction: ReactionInfo): String = sequence {
            for ((kind, deps) in reaction.allDependencies) {
                for (comp in deps) {
                    if (comp.isNotInjectedInReaction(kind, reaction)) continue

                    val borrow = comp.toBorrow(kind)
                    this.yield(borrow)
                }
            }
        }.toList().let {
            if (it.isEmpty()) ""
            else it.joinWithCommas(prefix = ", ")
        }

        return reactions.joinWithCommasLn(trailing = true) { n: ReactionInfo ->
            "${n.idx} => self.__impl.${n.workerId}(ctx${joinDependencies(n)})"
        }
    }

    /**
     * Number of reactions, including synthetic reactions.
     * Timers each have a reschedule and a bootstrap reaction.
     */
    private val ReactorInfo.totalNumReactions
        get() = 1 + reactions.size + 2 * timers.size

    /** Renders the branches corresponding to synthetic timer reactions in react_erased. */
    private fun ReactorInfo.syntheticTimerReactions(): String {
        fun ReactorInfo.timerReactionId(timer: TimerData, synthesisNum: Int) =
            reactions.size +
                    timers.indexOf(timer).also { assert(it != -1) } +
                    synthesisNum * timers.size // offset it by a block

        val branches = timers.map {
            "${timerReactionId(it, 0)} => ctx.reschedule_timer(&mut self.${it.rustFieldName})"
        } + timers.map {
            "${timerReactionId(it, 1)} => ctx.bootstrap_timer(&mut self.${it.rustFieldName})"
        }
        return branches.joinWithCommasLn(trailing = true)
    }

    /** Build the dependency graph using the assembler. */
    private fun ReactorInfo.graphDependencyDeclarations(): String {
        val reactions = reactions.map { n ->
            val deps: List<String> = mutableListOf<String>().apply {
                this += n.triggers.map { trigger -> "__assembler.declare_triggers(__self.${trigger.rustFieldName}.get_id(), ${n.invokerId})?;" }
                if (n.isStartup)
                    this += "__assembler.declare_triggers($rsRuntime::assembly::TriggerId::STARTUP, ${n.invokerId})?;"
                if (n.isShutdown)
                    this += "__assembler.declare_triggers($rsRuntime::assembly::TriggerId::SHUTDOWN, ${n.invokerId})?;"
                this += n.uses.map { trigger -> "__assembler.declare_uses(${n.invokerId}, __self.${trigger.rustFieldName}.get_id())?;" }
                this += n.effects.filterIsInstance<PortData>().map { port ->
                    // todo how is a bank channel represented in PortData?
                    //    reaction(startup) -> out[0]
                    if (port.isMultiport) {
                        "__assembler.effects_bank(${n.invokerId}, &__self.${port.rustFieldName})?;"
                    } else {
                        "__assembler.effects_port(${n.invokerId}, &__self.${port.rustFieldName})?;"
                    }
                }
            }

            n.loc.lfTextComment() + "\n" + deps.joinLn()
        }.joinLn()

        val timers = timers.flatMap {
            listOf(
                "__assembler.declare_triggers(__self.${it.rustFieldName}.get_id(), ${it.rescheduleReactionId})?;",
                // start reactions may "trigger" the timer, otherwise it schedules it
                "__assembler.declare_triggers($rsRuntime::assembly::TriggerId::STARTUP, ${it.startReactionId})?;",
                "__assembler.effects_timer(${it.startReactionId}, &__self.${it.rustFieldName})?;",
            )
        }.joinLn()

        return (reactions + "\n\n" + timers).trimEnd()
    }

    /** Name of the reschedule reaction for this timer. */
    private val TimerData.rescheduleReactionId: String
        get() = "__timer_schedule_$lfName"

    /** Name of the bootstrap reaction for this timer. */
    private val TimerData.startReactionId: String
        get() = "__timer_start_$lfName"

    private fun Emitter.makeMainFile(gen: GenerationInfo) {
        val mainReactor = gen.mainReactor
        val mainReactorNames = mainReactor.names
        this += with(PrependOperator) {
            """
            |${generatedByComment("//")}
            |#![allow(unused_imports)]
            |#![allow(non_snake_case)]
            |
            |extern crate env_logger;
            |#[macro_use]
            |extern crate log;
            |
            |// user dependencies
${"         |"..gen.crate.dependencies.keys.joinToString("\n") { "extern crate ${it.replace('-', '_')};" }}
            |
            |// user-defined modules
${"         |"..gen.crate.modulesToIncludeInMain.joinToString("\n") { "mod ${it.fileName.toString().removeSuffix(".rs")};" }}
            |
            |use $rsRuntime::*;
            |use log::LevelFilter;
            |pub use self::reactors::${mainReactorNames.wrapperName} as __MainReactor;
            |pub use self::reactors::${mainReactorNames.paramStructName} as __MainParams;
            |
            |struct CliParseResult(SchedulerOptions, __MainParams, Option<::log::LevelFilter>);
            |
            |fn main() {
            |    let CliParseResult(options, main_args, log_level) = cli::parse();
            |
            |    init_logger(log_level);
            |
            |    SyncScheduler::run_main::<__MainReactor>(options, main_args);
            |}
            |
            |fn init_logger(level: Option<LevelFilter>) {
            |    let mut builder = env_logger::Builder::from_env(env_logger::Env::default());
            |    builder.format_target(false);
            |    if let Some(level) = level {
            |       builder.filter_level(level);
            |    }
            |    builder.init();
            |}
            |
        """.trimMargin()
        }

        skipLines(2)

        makeCliModule(gen)

        skipLines(2)

        if (gen.properties.singleFile) {
            makeSingleFileProject(gen)
        } else {
            this += "mod reactors;\n"
        }
    }

    private fun Emitter.makeCliModule(gen:GenerationInfo) {
        val mainReactor = gen.mainReactor

        val defaultTimeOutAsStr = gen.properties.timeoutLf?.toString() ?: "0"
        val defaultTimeOutAsRust = gen.properties.timeoutLf?.toRustTimeExpr().toRustOption()

        this += """
            |#[cfg(not(feature="cli"))]
            |mod cli {
            |    use $rsRuntime::*;
            |    use super::*;
            |
            |    /// Fallback implementation which doesn't parse parameters.
            |    pub(super) fn parse() -> CliParseResult {
            |        if std::env::args().len() > 1 {
            |           warn!("CLI arguments are ignored, as the program was built without the \"cli\" feature.");
            |           warn!("In Lingua Franca, use the target property `cargo-features: [\"cli\"]`.");
            |           warn!("Proceeding with defaults defined at compile time.");
            |        }
            |
            |        let options = SchedulerOptions {
            |           timeout: $defaultTimeOutAsRust,
            |           keep_alive: ${gen.properties.keepAlive},
            |           threads: ${gen.properties.threads}, // note: zero means "1 per core"
            |        };

            |        // main params are entirely defaulted
            |        let main_args = __MainParams::new(
${"         |           "..mainReactor.ctorParams.joinWithCommasLn { (it.defaultValue ?: "Default::default()") }}
            |        );
            |
            |        CliParseResult(options, main_args, None)
            |    }
            |}
            |
            |#[cfg(feature="cli")]
            |mod cli {
            |    use $rsRuntime::*;
            |    use super::*;
            |    use clap::Parser;
            |    use std::str::FromStr;
            |
            |
            |    // these aliases are needed because clap interprets literal
            |    // occurrences of bool and Option.
            |    type BoolAlias = bool;
            |    type OptionAlias<T> = Option<T>;
            |
            |    #[derive(Debug, Parser)]
            |    #[clap(name = "${gen.executableName}")]
            |    struct Opt {
            |
            |        /// Whether to keep the program alive when the event queue is empty.
            |        /// This is only useful when physical actions are used, as they may
            |        /// push new asynchronous events.
            |        #[clap(long, default_value="${gen.properties.keepAlive}", help_heading=Some("RUNTIME OPTIONS"), value_name("bool"),)]
            |        keep_alive: BoolAlias,
            |
            |        /// Timeout for the program. A value of zero means no timeout. The
            |        /// timeout is in logical time, it means, no event past this tag will
            |        /// be processed. Notice that the program may shutdown earlier because
            |        /// of a call to request_stop.
            |        #[clap(long, default_value="$defaultTimeOutAsStr", parse(try_from_str = try_parse_duration), help_heading=Some("RUNTIME OPTIONS"), value_name("time"),)]
            |        timeout: OptionAlias<Duration>,
            |
            |        /// Number of threads to use to execute reactions in parallel. A value
            |        /// of zero means that the runtime will select a value depending on the
            |        /// number of cores available on the machine.
            |        /// This option is **ignored** unless the runtime crate has been built
            |        /// with the feature `parallel-runtime`.
            |        #[clap(long, default_value="${gen.properties.threads}", help_heading=Some("RUNTIME OPTIONS"), value_name("usize"),)]
            |        threads: usize,
            |
            |        /// Minimum logging level for the runtime. Specifying the log level on the
            |        /// command-line overrides the environment variable RUST_LOG. Note that release
            |        /// builds of the runtime are stripped of trace and debug logs, so only up
            |        /// to info can be enabled then. To trace programs, use a debug build.
            |        #[clap(long,
            |           possible_values(&["trace", "debug", "info", "warn", "error", "off"]),
            |           env = "RUST_LOG",
            |           hide_env_values = true,
            |           default_value="warn",
            |           value_name("level"),
            |           help_heading=Some("RUNTIME OPTIONS"),
            |        )]
            |        log_level: LevelFilter,
            |
${"         |        "..mainReactor.ctorParams.joinWithCommasLn { it.toCliParam() }}
            |    }
            |
            |    pub(super) fn parse() -> CliParseResult {
            |        let opts = Opt::parse();
            |
            |        let options = SchedulerOptions {
            |            timeout: opts.timeout,
            |            keep_alive: opts.keep_alive,
            |            threads: opts.threads,
            |        };
            |
            |        let main_args = __MainParams::new(
${"         |           "..mainReactor.ctorParams.joinWithCommasLn { "opts." + it.cliParamName }}
            |        );
            |
            |        // Note here we return always Some(LogLevel), because clap supports reading RUST_LOG env var
            |        // Note that there is an inconsistency here, as clap will just parse the simple values,
            |        // while env_logger supports more complicated patterns for this env var.
            |
            |        CliParseResult(options, main_args, Some(opts.log_level))
            |    }
            |
            |    fn try_parse_duration(t: &str) -> ::std::result::Result<Option<Duration>, String> {
            |        reactor_rt::try_parse_duration(t).map(|d| {
            |            if d.is_zero() { None } else { Some(d) }
            |        })
            |    }
            |}
        """.trimMargin()
    }

    private val CtorParamInfo.cliParamName get() = "main_" + lfName.camelToSnakeCase()

    private fun CtorParamInfo.toCliParam() = buildString {
        documentation?.lines()?.map { "///$it" }?.forEach { appendLine(it) }
        append("#[clap(long, help_heading=Some(\"MAIN REACTOR PARAMETERS\"), ")

        if (isList) // todo this should be supported but test it
            throw UnsupportedGeneratorFeatureException("main parameters with list types")

        if (defaultValueAsTimeValue != null)
            append("default_value=\"").append(defaultValueAsTimeValue).append("\", ")
        else if (defaultValue != null)
            append("default_value=\"").append(defaultValue.removeSurrounding("\"")).append("\", ")
        else
            append("required=true, ")

        if (isTime) {
            append("parse(try_from_str = $rsRuntime::try_parse_duration), value_name(\"time\"),")
        } else {
            append("value_name(\"${type.escapeStringLiteral()}\"),")
        }

        appendLine(")]")
        append(cliParamName).append(": ").append(type)
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


    /**
     * Rust pattern that deconstructs a ctor param tuple into individual variables.
     * Note that if all variables are in scope, it may also be used as a constructor.
     */
    private val ReactorInfo.ctorParamsDeconstructor: TargetCode
        get() {
            val fields = ctorParams.joinWithCommas { it.lfName.escapeRustIdent() }
            return "${names.paramStructName} {  __phantom, $fields }"
        }

    /** Renders the field of the reactor struct that will contain this component. */
    private fun ReactorComponent.toStructField(): TargetCode {
        val fieldVisibility = if (this is PortData) "pub " else ""
        return "$fieldVisibility$rustFieldName: ${toType()}"
    }

    /** The owned type of this reactor component (type of the struct field). */
    private fun ReactorComponent.toType(): TargetCode = when (this) {
        is ActionData                      ->
            if (isLogical) "$rsRuntime::LogicalAction<${dataType ?: "()"}>"
            else "$rsRuntime::PhysicalActionRef<${dataType ?: "()"}>"
        is PortData, is ChildPortReference -> with(this as PortLike) {
            if (isMultiport) "$rsRuntime::PortBank<$dataType>"
            else "$rsRuntime::Port<$dataType>"
        }
        is TimerData                       -> "$rsRuntime::Timer"
    }

    /** Initial expression for the field. */

    private fun ReactorComponent.initialExpression(): TargetCode = when (this) {
        is ActionData         -> {
            val delay = minDelay.toRustOption()
            val ctorName = if (isLogical) "new_logical_action" else "new_physical_action"
            "__assembler.$ctorName::<$dataType>(\"$lfName\", $delay)"
        }
        is TimerData          -> "__assembler.new_timer(\"$lfName\", $offset, $period)"
        is PortData           -> {
            if (widthSpec != null) {
                "__assembler.new_port_bank::<$dataType>(\"$lfName\", $isInput, $widthSpec)?"
            } else {
                "__assembler.new_port::<$dataType>(\"$lfName\", $isInput)"
            }
        }
        is ChildPortReference -> {
            if (isMultiport) {
                throw UnsupportedGeneratorFeatureException("Multiport references from parent reactor")
            } else {
                "__assembler.new_port::<$dataType>(\"$childName.$lfName\", $isInput)"
            }
        }
    }

    /** The type of the parameter injected into a reaction for the given dependency. */
    private fun ReactorComponent.toBorrowedType(kind: DepKind): TargetCode {
        fun portRefWrapper(dataType: TargetCode, isMultiport: Boolean) =
            when {
                kind == DepKind.Effects && isMultiport -> "$rsRuntime::WritablePortBank<$dataType>" // note: owned
                kind == DepKind.Effects                -> "$rsRuntime::WritablePort<$dataType>" // note: owned
                isMultiport                            -> "$rsRuntime::ReadablePortBank<$dataType>" // note: owned
                else                                   -> "&$rsRuntime::ReadablePort<$dataType>" // note: a reference
            }

        return when (this) {
            is PortData           -> portRefWrapper(dataType, isMultiport)
            is ChildPortReference -> portRefWrapper(dataType, isMultiport)
            is TimerData          -> "&${toType()}"
            is ActionData         -> if (isLogical && kind == DepKind.Effects) "&mut ${toType()}" else "&${toType()}"
        }
    }

    /**
     * Produce an instance of the borrowed type ([toBorrowedType]) to inject
     * into a reaction. This conceptually just borrows the field.
     */
    private fun ReactorComponent.toBorrow(kind: DepKind): TargetCode {
        fun ReactorComponent.portBorrow(kind: DepKind, isMultiport: Boolean) =
            when {
                kind == DepKind.Effects && isMultiport  -> "$rsRuntime::WritablePortBank::new(&mut self.$rustFieldName)" // note: owned
                kind == DepKind.Effects && !isMultiport -> "$rsRuntime::WritablePort::new(&mut self.$rustFieldName)" // note: owned
                isMultiport                             -> "$rsRuntime::ReadablePortBank::new(&self.$rustFieldName)" // note: owned
                else                                    -> "&$rsRuntime::ReadablePort::new(&self.$rustFieldName)" // note: a reference
            }

        return when (this) {
            is PortData           -> portBorrow(kind, isMultiport)
            is ChildPortReference -> portBorrow(kind, isMultiport)
            is ActionData         -> if (kind == DepKind.Effects) "&mut self.$rustFieldName" else "&self.$rustFieldName"
            is TimerData          -> "&self.$rustFieldName"
        }
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


    /** Action that must be taken to cleanup this component within the `cleanup_tag` procedure. */
    private fun ReactorComponent.cleanupAction(): TargetCode? = when (this) {
        is ActionData         ->
            if (isLogical) "ctx.cleanup_logical_action(&mut self.$rustFieldName);"
            else "ctx.cleanup_physical_action(&mut self.$rustFieldName);"
        is PortLike           ->
            if (isMultiport) "ctx.cleanup_multiport(&mut self.$rustFieldName);"
            else "ctx.cleanup_port(&mut self.$rustFieldName);"
        is ChildPortReference -> "ctx.cleanup_port(&mut self.$rustFieldName);"
        else                  -> null
    }

    /** Renders the definition of the worker function generated for this reaction. */
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
