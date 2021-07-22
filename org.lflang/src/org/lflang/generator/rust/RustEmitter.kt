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

import org.lflang.generator.PrependOperator
import org.lflang.generator.rust.RustEmitter.generateRustProject
import org.lflang.generator.rust.RustEmitter.rsRuntime
import org.lflang.joinLines
import org.lflang.joinWithCommas
import org.lflang.withDQuotes
import java.nio.file.Files
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
        fileConfig.emit("src/main.rs") { makeMainFile(gen) }
        fileConfig.emit("src/reactors/mod.rs") { makeReactorsAggregateModule(gen) }
        for (reactor in gen.reactors) {
            fileConfig.emit("src/reactors/${reactor.names.modName}.rs") {
                makeReactorModule(reactor)
            }
        }

    }

    private fun Emitter.makeReactorModule(reactor: ReactorInfo) {
        this += with(reactor) {
            with (reactor.names) {
            with(ReactorComponentEmitter) {
                with(PrependOperator) {
                    """
                |${generatedByComment("//")}
                |#![allow(unused)]
                |
                |use std::sync::{Arc, Mutex};
                |use $rsRuntime::{LogicalInstant, PhysicalInstant, Duration};
                |use $rsRuntime::Offset::{After, Asap};
                |
${"             |"..reactor.preambles.joinToString("\n\n") { "// preamble {=\n${it.trimIndent()}\n// =}" }}
                |
                |// todo link to source
                |pub struct $structName {
${"             |    "..reactor.stateVars.joinToString(",\n") { it.lfName + ": " + it.type }}
                |}
                |
                |#[warn(unused)]
                |impl $structName {
                |
${"             |    "..reactions.joinToString("\n\n") { it.toWorkerFunction(reactor) }}
                |
                |}
                |
                |/// Parameters for the construction of a [$structName]
                |#[derive(Clone)]
                |pub struct ${names.paramStructName} {
${"             |    "..ctorParams.joinToString(",\n") { "pub ${it.lfName}: ${it.type}" }}
                |}
                |
                |
                |//------------------------//
                |
                |
                |pub struct $dispatcherName {
                |    // state struct
                |    _impl: $structName,
                |    // ctor parameters
                |    _params: $paramStructName,
                |    // other components
${"             |    "..otherComponents.joinToString("\n") { it.toStructField() + "," }}
                |}
                |
                |
                |reaction_ids!(
                |  ${reactions.joinToString(", ", "pub enum $reactionIdName {", "}") { it.rustId }}
                |);
                |
                |impl $rsRuntime::ReactorDispatcher for $dispatcherName {
                |    type ReactionId = $reactionIdName;
                |    type Wrapped = $structName;
                |    type Params = $paramStructName;
                |
                |    #[inline]
                |    fn assemble(params: Self::Params) -> Self {
                |        let $ctorParamsDeconstructor = params.clone();
                |        Self {
                |            _params: params,
                |            _impl: $structName {
${"             |                "..reactor.stateVars.joinToString(",\n") { it.lfName + ": " + (it.init ?: "Default::default()") }}
                |            },
${"             |            "..otherComponents.joinToString(",\n") { it.lfName + ": " + it.initialExpression() }}
                |        }
                |    }
                |
                |    fn react(&mut self, ctx: &mut $rsRuntime::LogicalCtx, rid: Self::ReactionId) {
                |        match rid {
${"             |            "..reactionWrappers(reactor)}
                |        }
                |    }
                |}
                |
                |
                |//------------------------//
                |
                |use $rsRuntime::*; // after this point there's no user-written code
                |
                |
                |pub struct $assemblerName {
                |    pub(in super) _rstate: Arc<Mutex<$dispatcherName>>,
                |    // nested reactors
${"             |    "..nestedInstances.joinToString("\n") { it.toStructField() + "," }}
                |    // self reactions
${"             |    "..reactor.reactions.joinToString(",\n") { it.invokerFieldDeclaration() }}
                |}
                |
                |impl ReactorAssembler for $assemblerName {
                |    type RState = $dispatcherName;
                |
                |    #[inline]
                |    fn start(&mut self, startup_ctx: &mut StartupCtx) {
                |        if ${reactor.hasSelfStartupLogic()} {
                |            // Startup this reactor
                |            let dispatcher = &mut self._rstate.lock().unwrap();
                |            let ctx = &mut startup_ctx.logical_ctx();
                |
                |            // Startup timers
${"             |            "..reactor.timers.joinToString("\n" ) {
                        "startup_ctx.start_timer(&dispatcher.${it.lfName});"
}}
                |            // Execute reactions triggered by startup in order. FIXME use enqueue
${"             |            "..reactor.reactions.filter { it.isStartup }.joinToString("\n") { 
                        "dispatcher.react(ctx, $reactionIdName::${it.rustId});" 
}}
                |        }
                |        // Startup children reactors
${"             |        "..nestedInstances.joinToString("\n") { "self.${it.lfName}.start(startup_ctx);" }}
                |    }
                |
                |    fn assemble(
                |       reactor_id: &mut ReactorId, 
                |       args: <Self::RState as ReactorDispatcher>::Params
                |    ) -> Self {
                |        let mut _rstate = Arc::new(Mutex::new(Self::RState::assemble(args)));
                |        let this_reactor = reactor_id.get_and_increment();
                |        let mut reaction_id = 0;
                |
${"             |        "..reactor.reactions.joinToString("\n") { it.reactionInvokerInitializer() }}
                |
${"             |        "..assembleChildReactors()}
                |
                |        if ${reactor.otherComponents.any()} {
                |            // Declare local dependencies
                |            let mut statemut = _rstate.lock().unwrap();
                |
${"             |           "..localDependencyDeclarations(reactor)}
                |        }
                |        {
${"             |            "..declareChildConnections()}
                |        }
                |        Self {
                |            _rstate,
${"             |            "..nestedInstances.joinToString("\n") { it.lfName + "," }}
${"             |            "..reactions.joinToString("\n") { it.invokerId + "," }}
                |        }
                |    }
                |}
        """.trimMargin()
                }
            }
            }
        }
    }

    private fun ReactorInfo.hasSelfStartupLogic() =
        reactions.any { it.isStartup } || otherComponents.any { it is TimerData }

    private fun ReactorInfo.assembleChildReactors(): String {
        fun NestedReactorInstance.paramStruct(): String =
            args.entries.joinToString(", ", "super::${names.paramStructName} { ", " }") {
                it.key + ": " + it.value
            }

        return nestedInstances.joinToString("\n") {
            """
                    ${it.loc.lfTextComment()}
                    let mut ${it.lfName} = super::${it.names.assemblerName}::assemble(reactor_id, ${it.paramStruct()});
                """.trimIndent()
        }
    }


    private fun ReactorInfo.declareChildConnections(): String {
        val declarations = nestedInstances.joinToString("\n") {
            "let mut ${it.lfName} = ${it.lfName}._rstate.lock().unwrap();"
        }

        return declarations + "\n" +
                connections.joinToString("\n", "// Declare connections\n") {
                    it.locationInfo().lfTextComment() + "\n" +
                            PortEmitter.declareConnection(it)
                }
    }

    private fun reactionWrappers(reactor: ReactorInfo): String {

        fun joinDependencies(n: ReactionInfo): String =
            n.allDependencies
                .takeIf { it.isNotEmpty() }
                ?.mapNotNull { with(ReactorComponentEmitter) { it.toBorrow() } }
                ?.joinToString(", ", prefix = ", ")
                .orEmpty()

        return reactor.reactions.joinToString { n: ReactionInfo ->
            """
                ${reactor.names.reactionIdName}::${n.rustId} => {
                    self._impl.${n.workerId}(ctx, &self._params${joinDependencies(n)})
                }
            """
        }
    }

    private fun localDependencyDeclarations(reactor: ReactorInfo): String {
        fun allDownstreamDeps(component: ReactorComponent) =
            reactor.influencedReactionsOf(component).map {
                it.invokerId + ".clone()"
            }.let { base ->
                if (component is TimerData) base + "reschedule_self_timer!(this_reactor, _rstate, 1000)"
                else base
            }

        fun vecLiteral(list: List<String>) =
            list.joinToString(", ", "vec![", "]")

        return reactor.otherComponents.joinToString("\n") {
            "statemut.${it.lfName}.set_downstream(${vecLiteral(allDownstreamDeps(it))}.into());"
        }
    }

    /**
     * Returns a list of the reactions which need to be triggered
     * when the [component] is set at a specific time step. Eg if
     * the component is a port, the reactions to trigger are all
     * those which have registered a dependency on that port.
     */
    private fun ReactorInfo.influencedReactionsOf(component: ReactorComponent): List<ReactionInfo> =
        reactions.filter {
            component in it.triggers
        }


    private fun Emitter.makeMainFile(gen: GenerationInfo) {
        val mainReactor = gen.mainReactor.names
        this += """
            |${generatedByComment("//")}
            |#![allow(unused_imports)]
            |#![allow(non_snake_case)]
            |
            |#[macro_use]
            |extern crate $runtimeCrateFullName;
            |
            |mod reactors;
            |
            |use $rsRuntime::*;
            |use self::reactors::${mainReactor.assemblerName} as _MainAssembler;
            |use self::reactors::${mainReactor.paramStructName} as _MainParams;
            |
            |fn main() {
            |    let mut reactor_id = ReactorId::first();
            |    let mut topcell = <_MainAssembler as ReactorAssembler>::assemble(&mut reactor_id, _MainParams {/* main params are de facto forbidden */});
            |    let options = SchedulerOptions {
            |       timeout: ${gen.properties.timeout.toRustOption()},
            |       keep_alive: ${gen.properties.keepAlive}
            |    };
            |    let mut scheduler = SyncScheduler::new(options);
            |    scheduler.startup(|mut starter| {
            |        topcell.start(&mut starter);
            |    });
            |    scheduler.launch_async().join().unwrap();
            |}
        """.trimMargin()
    }

    private fun Emitter.makeReactorsAggregateModule(gen: GenerationInfo) {
        fun ReactorInfo.modDecl(): String = with(names) {
            // We make some declarations public to be able to refer to them
            // simply when building nested reactors.
            """
                mod $modName;
                pub use self::$modName::$assemblerName;
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

            |# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

            |[dependencies.$runtimeCrateFullName]
            |#-- The reactor runtime --#
            |${runtimeCrateLocation()}
            |
            |[[bin]]
            |name = "${gen.executableName}"
            |path = "src/main.rs"
        """.trimMargin()
    }


    /**
     * Select location of runtime crate for LFC. It may be fetched
     * from Git or from your PC if you have a local development version.
     * You need to set `RUST_REACTOR_RT_PATH` to a correct value
     */
    private fun runtimeCrateLocation(): String {
        val path = System.getenv("RUST_REACTOR_RT_PATH")
        return when {
            path == null                       -> "git = \"ssh://git@github.com/icyphy/reactor-rust.git\""
            Files.isDirectory(Paths.get(path)) -> "path = \"${Paths.get(path).toAbsolutePath()}\""
            else                               -> throw IllegalStateException("Not a directory: $path")
        }
    }


    /// Rust pattern that deconstructs a ctor param tuple into individual variables
    private val ReactorInfo.ctorParamsDeconstructor: TargetCode
        get() = "${names.paramStructName} { ${ctorParams.joinToString(", ") { it.lfName }} }"
}


private object ReactorComponentEmitter {


    /**
     * Returns null if there is no need to manipulate the
     * dependency within the reaction.
     */
    fun ReactorComponent.toBorrow(): TargetCode? = when (this) {
        is PortData   ->
            if (isInput) "&self.$lfName"
            else "&mut self.$lfName"
        is ActionData -> "&self.$lfName"
        is TimerData  -> null
    }

    fun ReactorComponent.isInjectedInReaction(): Boolean =
        this !is TimerData

    fun ReactorComponent.toBorrowedType(): TargetCode =
        if (this is PortData && !this.isInput) "&mut ${toType()}"
        else "& ${toType()}"

    fun ReactorComponent.toType(): TargetCode = when (this) {
        is ActionData ->
            if (isLogical) "$rsRuntime::LogicalAction"
            else "$rsRuntime::PhysicalAction"
        is PortData   ->
            if (isInput) "$rsRuntime::InputPort<$dataType>"
            else "$rsRuntime::OutputPort<$dataType>"
        is TimerData  -> "$rsRuntime::Timer"
    }

    fun ReactorComponent.initialExpression(): TargetCode = when (this) {
        is ActionData -> {
            val delay = minDelay.toRustOption()
            toType() + "::new(${lfName.withDQuotes()}, $delay)"
        }
        is TimerData  -> toType() + "::new(${lfName.withDQuotes()}, $offset, $period)"
        // todo missing name for Ports
        else          -> "Default::default()"
    }


    fun ReactorComponent.toStructField(): TargetCode {
        val fieldVisibility = if (this is PortData) "pub " else ""

        return "$fieldVisibility$lfName: ${toType()}"
    }


    fun NestedReactorInstance.toStructField() =
        "$lfName: ${names.modulePath}::${names.assemblerName}" // Arc<Mutex<${names.modulePath}::${names.dispatcherName}>>

    fun ReactionInfo.reactionInvokerInitializer() =
        "let $invokerId = new_reaction!(this_reactor, reaction_id, _rstate, $rustId);"

    fun ReactionInfo.invokerFieldDeclaration() =
        "$invokerId: Arc<$rsRuntime::ReactionInvoker>"

    fun ReactionInfo.toWorkerFunction(reactor: ReactorInfo): String {
        fun ReactionInfo.reactionParams() =
            allDependencies
                .filter { it.isInjectedInReaction() }
                .joinToString(",\n") { d ->
                    "${d.lfName}: ${d.toBorrowedType()}".let { str ->
                        if (d !is PortData) "#[allow(unused)] $str" else str
                    }
                }

        val indent = " ".repeat("fn $workerId(".length)
        return with(PrependOperator) {
            """
                |${loc.lfTextComment()}
                |fn $workerId(&mut self, 
                |$indent#[allow(unused)] ctx: &mut $rsRuntime::LogicalCtx,
                |$indent#[allow(unused)] params: &${reactor.names.paramStructName},
${"             |$indent"..reactionParams()}) {
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
