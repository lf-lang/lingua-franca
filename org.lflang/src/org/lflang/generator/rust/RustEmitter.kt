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
import org.lflang.joinWithCommas
import org.lflang.withDQuotes

/**
 * Generates Rust code
 */
object RustEmitter {
    /**
     * Name given to the Rust runtime crate within generated code,
     * can be used to qualify names.
     */
    const val rsRuntime = "_rr"

    fun generateFiles(fileConfig: RustFileConfig, gen: GenerationInfo) {

        fileConfig.emit("Cargo.toml") { makeCargoFile(gen) }
        fileConfig.emit("src/bin/main.rs") { makeMainFile(gen) }
        fileConfig.emit("src/lib.rs") { makeRootLibFile(gen) }
        for (reactor in gen.reactors) {
            fileConfig.emit("src/${reactor.modName}.rs") {
                makeReactorModule(reactor)
            }
        }

    }

    private fun Emitter.makeReactorModule(reactor: ReactorInfo) {
        this += with(reactor) {
            with(ReactorComponentEmitter) {
                with(PrependOperator) {
                    """
                |// ${generatedByHeader()}
                |use std::sync::{Arc, Mutex};
                |
                |// todo link to source
                |struct $structName {
                |    // TODO state vars
                |}
                |
                |impl $structName {
                |
                |    // TODO change this generation logic. There may be several reactions invoked on startup
                |    fn react_startup(link: $rsRuntime::SchedulerLink, ctx: &mut $rsRuntime::LogicalCtx) {
${"             |       "..reactions.firstOrNull { it.isStartup }?.body.orEmpty()}
                |    }
                |
${"             |    "..reactions.joinToString("\n\n") { it.toWorkerFunction() }}
                |
                |}
                |
                |struct $dispatcherName {
                |    _impl: $structName,
${"             |    "..otherComponents.joinToString(",\n") { it.toStructField() }}
                |}
                |
                |
                |reaction_ids!(
                |  ${reactions.joinToString(", ", "enum $reactionIdName {", "}") { it.rustId }}
                |);
                |
                |impl $rsRuntime::ReactorDispatcher for $dispatcherName {
                |    type ReactionId = $reactionIdName;
                |    type Wrapped = $structName;
                |    type Params = (${ctorParamTypes.joinWithCommas()});
                |
                |
                |    fn assemble(_params: Self::Params) -> Self {
                |        Self {
                |            _impl: $structName {/*todo*/},
${"             |            "..otherComponents.joinToString(",\n") { it.toFieldInitializer() }}
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
                |struct $assemblerName {
                |   _rstate: Arc<Mutex<$dispatcherName>>,
${"             |   "..reactor.reactions.joinToString(",\n") { it.invokerFieldDeclaration() }}
                |}
                |
                |impl $rsRuntime::ReactorAssembler for $assemblerName {
                |    type RState = $dispatcherName;
                |
                |    fn start(&mut self, link: $rsRuntime::SchedulerLink, ctx: &mut $rsRuntime::LogicalCtx) {
                |       let state = &self._rstate.lock().unwrap();
                |       $structName::react_startup(link, ctx, /*todo dependencies*/);
                |    }
                |
                |    fn assemble(reactor_id: &mut $rsRuntime::ReactorId, args: <Self::RState as $rsRuntime::ReactorDispatcher>::Params) -> Self {
                |       let mut _rstate = Arc::new(Mutex::new(<Self::RState as $rsRuntime::ReactorDispatcher>::assemble(args)));
                |       let this_reactor = reactor_id.get_and_increment();
                |       let mut reaction_id = 0;
                |
${"             |       "..reactor.reactions.joinToString("\n") { it.reactionInvokerInitializer() }}
                |
                |       { // declare local dependencies
                |           let mut statemut = _rstate.lock().unwrap();
                |
${"             |           "..localDependencyDeclarations(reactor)}
                |       }
                |
                |       Self {
                |           _rstate,
${"             |           "..reactions.joinToString(",\n") { it.invokerId }}
                |       }
                |    }
                |}
        """.trimMargin()
                }
            }
        }
    }

    private fun reactionWrappers(reactor: ReactorInfo): String {

        fun joinDependencies(n: ReactionInfo): String =
            if (n.depends.isEmpty()) ""
            else n.depends.joinToString(", ", prefix = ", ") {
                with(ReactorComponentEmitter) { it.toBorrow() }
            }

        return reactor.reactions.joinToString { n: ReactionInfo ->
            """
                ${reactor.reactionIdName}::${n.rustId} => {
                    self._impl.${n.workerId}(ctx${joinDependencies(n)})
                }
            """
        }
    }

    private fun localDependencyDeclarations(reactor: ReactorInfo): String {
        fun vecOfReactions(list: List<ReactionInfo>) =
            list.joinToString(", ", "vec![", "]") { it.invokerId + ".clone()" }

        return reactor.otherComponents.joinToString(",\n") {
            "statemut.${it.name}.set_downstream(${vecOfReactions(reactor.influencedReactionsOf(it))}.into());"
        }
    }

    private fun ReactorInfo.influencedReactionsOf(component: ReactorComponent): List<ReactionInfo> =
        // todo transitive closure
        reactions.filter {
            component in it.depends
        }


    private fun Emitter.makeMainFile(gen: GenerationInfo) {
        this += """
            |// ${generatedByHeader()}
            |#[allow(unused_imports)]
            |#[macro_use]
            |extern crate ${gen.crate.name} as $rsRuntime;
            |
            |fn main() {
            | // todo
            |
            |}
        """.trimMargin()
    }

    private fun Emitter.makeRootLibFile(gen: GenerationInfo) {
        this += with(PrependOperator) {
            """
            |// ${generatedByHeader()}
            |//! Root of this crate
            |#[macro_use]
            |extern crate reactor_rust as $rsRuntime;
            |
${"         |"..gen.reactors.joinToString("\n") { "mod ${it.modName};" }}
            |
        """.trimMargin()
        }
    }

    private fun Emitter.makeCargoFile(gen: GenerationInfo) {
        val (crate, _) = gen
        this += """
            |#-- ${generatedByHeader()} --#
            |[package]
            |name = "${crate.name}"
            |version = "${crate.version}"
            |authors = [${crate.authors.joinToString(", ") { it.withDQuotes() }}]
            |edition = "2018"

            |# See more keys and their definitions at https://doc.rust-lang.org/cargo/reference/manifest.html

            |[dependencies.reactor_rust]
            |#-- The reactor runtime --#
            |# See https://doc.rust-lang.org/cargo/appendix/git-authentication.html#ssh-authentication
            |# git = "ssh://git@github.com:icyphy/reactor-rust.git"
            |path = "/home/clem/Documents/Cours/rust-reactors"
        """.trimMargin()
    }


    private fun generatedByHeader() = "Generated by LFC @ todo time"

}


private object ReactorComponentEmitter {


    fun ReactorComponent.toBorrow() = when (this) {
        is PortData   ->
            if (isInput) "&self.$name"
            else "&mut self.$name"
        is ActionData -> "&self.$name"
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

    fun ReactorComponent.toFieldInitializer() = when (this) {
        is ActionData -> toType() + " (None, ${name.withDQuotes()})"
        else          -> "Default::default()"
    }

    fun ReactorComponent.toStructField() =
        "$name: ${toType()}"

    fun ReactionInfo.reactionInvokerInitializer() =
        "let $invokerId = new_reaction!(this_reactor, reaction_id, _rstate, $rustId);"

    fun ReactionInfo.invokerFieldDeclaration() =
        "$invokerId: Arc<$rsRuntime::ReactionInvoker>"

    fun ReactionInfo.toWorkerFunction() =
        """
            // todo metadata
            fn ${this.workerId}(&mut self, ctx: &mut _rr::LogicalCtx, ${reactionParams()}) {
                ${this.body}
            }
        """.trimIndent()

    private fun ReactionInfo.reactionParams() =
        depends.joinToString(", ") { "${it.name}: ${it.toBorrowedType()}" }


}
