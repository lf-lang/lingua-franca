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
import org.lflang.generator.rust.RustEmitter.rsRuntime
import org.lflang.withDQuotes
import java.lang.IllegalStateException
import java.nio.file.Files
import java.nio.file.Paths
import java.time.LocalDateTime

import java.time.format.DateTimeFormatter




/**
 * Generates Rust code
 */
object RustEmitter {
    /** Name of the runtime crate that is in its Cargo.toml.*/
    const val runtimeCrateFullName = "reactor_rt"

    /** Qualification prefix to refer to a member of the runtime library crate. */
    const val rsRuntime = "::$runtimeCrateFullName"

    fun generateFiles(fileConfig: RustFileConfig, gen: GenerationInfo) {

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
                |
${"             |"..reactor.preambles.joinToString("\n\n") {"// preamble {=\n${it.trimIndent()}\n// =}"}}
                |
                |// todo link to source
                |pub struct $structName {
${"             |    "..reactor.stateVars.joinToString(",\n") { it.lfName + ": " + it.type }}
                |}
                |
                |impl $structName {
                |
${"             |    "..reactions.joinToString("\n\n") { it.toWorkerFunction() }}
                |
                |}
                |
                |pub struct $dispatcherName {
                |    // state struct
                |    _impl: $structName,
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
                |    type Params = $ctorParamsTupleType;
                |
                |
                |    fn assemble(_params: Self::Params) -> Self {
                |        Self {
                |            _impl: $structName {
${"             |                "..reactor.stateVars.joinToString(",\n") { it.lfName + ": " + (it.init ?: "Default::default()") }}
                |            },
${"             |            "..otherComponents.joinToString(",\n") {  it.lfName + ": " + it.initialExpression() }}
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
                |use $rsRuntime::*; // after this point there's no user-written code
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
                |    fn start(&mut self, startup_ctx: &mut StartupCtx) {
                |        if ${reactor.reactions.any { it.isStartup }} {
                |            // Startup this reactor
                |            let dispatcher = &mut self._rstate.lock().unwrap();
                |            let ctx = &mut startup_ctx.logical_ctx();
                |
                |            // Execute reactions triggered by startup in order.
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
                |       let mut _rstate = Arc::new(Mutex::new(Self::RState::assemble(args)));
                |       let this_reactor = reactor_id.get_and_increment();
                |       let mut reaction_id = 0;
                |
${"             |       "..reactor.reactions.joinToString("\n") { it.reactionInvokerInitializer() }}
                |
${"             |       "..assembleChildReactors()}
                |
                |       if ${reactor.otherComponents.any()} {
                |           // declare local dependencies
                |           let mut statemut = _rstate.lock().unwrap();
                |
${"             |           "..localDependencyDeclarations(reactor)}
                |       }
                |       {
                |           // declare connections between children
${"             |           "..declareChildConnections()}
                |       }
                |       Self {
                |           _rstate,
${"             |           "..nestedInstances.joinToString("\n") { it.lfName + "," }}
${"             |           "..reactions.joinToString("\n") { it.invokerId + "," }}
                |       }
                |    }
                |}
        """.trimMargin()
                }
            }
            }
        }
    }

    private fun ReactorInfo.assembleChildReactors(): String =
        nestedInstances.joinToString("\n") {
            "let mut ${it.lfName} = super::${it.names.assemblerName}::assemble(reactor_id, (/*todo params*/));"
        }


    private fun ReactorInfo.declareChildConnections(): String {
        val declarations = nestedInstances.joinToString("\n") {
            "let mut ${it.lfName} = ${it.lfName}._rstate.lock().unwrap();"
        }

        // todo bind_ports
        return declarations + "\n// TODO bind_ports(...)"
    }


    private fun reactionWrappers(reactor: ReactorInfo): String {

        fun joinDependencies(n: ReactionInfo): String =
            n.allDependencies
                .takeIf { it.isNotEmpty() }
                ?.joinToString(", ", prefix = ", ") {
                    with(ReactorComponentEmitter) { it.toBorrow() }
                }.orEmpty()

        return reactor.reactions.joinToString { n: ReactionInfo ->
            """
                ${reactor.names.reactionIdName}::${n.rustId} => {
                    self._impl.${n.workerId}(ctx${joinDependencies(n)})
                }
            """
        }
    }

    private fun localDependencyDeclarations(reactor: ReactorInfo): String {
        fun vecOfReactions(list: List<ReactionInfo>) =
            list.joinToString(", ", "vec![", "]") { it.invokerId + ".clone()" }

        return reactor.otherComponents.joinToString(",\n") {
            "statemut.${it.lfName}.set_downstream(${vecOfReactions(reactor.influencedReactionsOf(it))}.into());"
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
            |
            |fn main() {
            |    let mut reactor_id = ReactorId::first();
            |    let mut topcell = <self::reactors::${gen.mainReactor.names.assemblerName} as ReactorAssembler>::assemble(&mut reactor_id, (/*todo params*/));
            |    let options = SchedulerOptions {
            |       timeout: None,
            |       keep_alive: false
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
        fun ReactorNames.modDecl() =
            "mod $modName;\npub use self::$modName::$assemblerName;"

        this += with(PrependOperator) {
            """
            |${generatedByComment("//")}
            |
${"         |"..gen.reactors.joinToString("\n") { it.names.modDecl() }}
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

    private val timeFormatter = DateTimeFormatter.ofPattern("yyyy/MM/dd HH:mm:ss")
    private fun generatedByComment(delim: String) =
        "$delim-- Generated by LFC @ ${timeFormatter.format(LocalDateTime.now())} --$delim"
}


private object ReactorComponentEmitter {


    fun ReactorComponent.toBorrow() = when (this) {
        is PortData   ->
            if (isInput) "&self.$lfName"
            else "&mut self.$lfName"
        is ActionData -> "&self.$lfName"
    }

    fun ReactorComponent.toBorrowedType() =
        if (this is PortData && !this.isInput) "&mut ${toType()}"
        else "& ${toType()}"

    fun ReactorComponent.toType() = when (this) {
        is ActionData ->
            if (isLogical) "$rsRuntime::LogicalAction"
            else "$rsRuntime::PhysicalAction"
        is PortData   ->
            if (isInput) "$rsRuntime::InputPort<$dataType>"
            else "$rsRuntime::OutputPort<$dataType>"
    }

    fun ReactorComponent.initialExpression() = when (this) {
        is ActionData -> toType() + " (None, ${lfName.withDQuotes()})"
        else          -> "Default::default()"
    }

    fun ReactorComponent.toStructField(): String {
        val fieldVisibility = if (this is PortData) "pub " else ""

        return "$fieldVisibility$lfName: ${toType()}"
    }


    fun NestedReactorInstance.toStructField() =
        "$lfName: ${names.modulePath}::${names.assemblerName}" // Arc<Mutex<${names.modulePath}::${names.dispatcherName}>>

    fun ReactionInfo.reactionInvokerInitializer() =
        "let $invokerId = new_reaction!(this_reactor, reaction_id, _rstate, $rustId);"

    fun ReactionInfo.invokerFieldDeclaration() =
        "$invokerId: Arc<$rsRuntime::ReactionInvoker>"

    fun ReactionInfo.toWorkerFunction() =
        with(PrependOperator) {
            """
            |// todo reproduce header & metadata here
            |fn $workerId(&mut self, ctx: &mut $rsRuntime::LogicalCtx, ${reactionParams()}) {
${"         |    "..body}
            |}
        """.trimMargin()
        }

    private fun ReactionInfo.reactionParams() =
        allDependencies.joinToString(", ") { "${it.lfName}: ${it.toBorrowedType()}" }


}
