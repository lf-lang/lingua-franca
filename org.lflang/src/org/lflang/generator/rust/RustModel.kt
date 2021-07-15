/*
 * Copyright (c) 2021, TU Dresden.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

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
import org.lflang.generator.cpp.name
import org.lflang.generator.cpp.targetType
import org.lflang.lf.*
import java.util.*

private typealias Ident = String
private typealias TargetCode = String

/** Root model class for the entire generation. */
data class GenerationInfo(
    val crate: CrateInfo,
    val reactors: List<ReactorInfo>,
    val mainReactor: ReactorInfo, // it's also in the list
    val executableName: Ident
)

/**
 * Model class for a reactor class. This will be emitted as
 * several structs in a Rust module.
 */
data class ReactorInfo(
    /** Name of the reactor in LF. By LF conventions, this is a PascalCase identifier. */
    val lfName: Ident,
    /** Whether this is the main reactor. */
    val isMain: Boolean,
    /** A list of reactions, in order of their [ReactionInfo.idx]. */
    val reactions: List<ReactionInfo>,
    /** List of state variables. */
    val stateVars: List<StateVarInfo>,
    /** Other reactor components, like actions & timers. */
    val otherComponents: List<ReactorComponent>,
    /** List of ctor parameters. */
    val ctorParams: List<CtorParamInfo> = emptyList(),
    /** List of preambles, will be outputted at the top of the file. */
    val preambles: List<TargetCode>,

    val nestedInstances: List<NestedReactorInstance>

) {
    val names = ReactorNames(lfName)

    // Names of other implementation-detailistic structs.

    val assemblerName: Ident get() = names.assemblerName
    val reactionIdName: Ident get() = names.reactionIdName

    val ctorParamsTupleType: TargetCode
        get() = "(${ctorParams.map { it.type }.joinWithCommas()})"
}

class ReactorNames(private val lfName: Ident) {
    val modName: Ident = lfName.camelToSnakeCase()
    val structName: Ident get() = lfName
    val dispatcherName: Ident = "${structName}Dispatcher"
    val assemblerName: Ident = "${structName}Assembler"
    val reactionIdName: Ident = "${structName}Reactions"
}

data class NestedReactorInstance(
    val lfName: Ident,
    val reactorLfName: String,
    val params: ParamList
) {
    val names = ReactorNames(lfName)
}

sealed class ParamList {
    data class Positional(val list: List<TargetCode>) : ParamList()
    data class Named(val map: Map<Ident, TargetCode>) : ParamList()
}

/**
 * Model class for the parameters of a reactor constructor.
 */
data class CtorParamInfo(
    val lfName: Ident,
    val type: TargetCode,
    val defaultValue: (TargetCode)?
)

/** Model class for a state variable. */
data class StateVarInfo(
    /**
     * Identifier of the state var. From within a reaction
     * body, the state var is accessible as a field with type
     * [type] declared on the `self` struct.
     */
    val lfName: String,
    /** Rust static type of the struct field. Must be `Sized`. */
    val type: TargetCode,
    /**
     * The field initializer, a Rust expression. If null,
     * will default to `Default::default()`.
     */
    val init: (TargetCode)?
)

/**
 * Model class for a single reaction.
 */
data class ReactionInfo(
    /** Index in the containing reactor. */
    val idx: Int,
    /** Target code for the reaction body. */
    val body: TargetCode,
    /** Dependencies declared by the reaction, which are served to the worker function. */
    val depends: Set<ReactorComponent>,
    /** Whether the reaction is triggered by the startup event. */
    val isStartup: Boolean,
    /** Whether the reaction is triggered by the shutdown event. */
    val isShutdown: Boolean,
) {
    // those are implementation details

    /** The ID of the reaction in the reaction enum. */
    val rustId: String = "R$idx"

    /** The name of the worker function for this reaction. */
    val workerId: String = "react_$idx"

    /** The name of the `ReactionInvoker` field for this reaction. */
    val invokerId: String = "react_$idx"
}

/**
 * Metadata about the generated crate. Mostly doesn't matter.
 */
data class CrateInfo(
    /** Name of the crate. According to Rust conventions this should be a snake_case name. */
    val name: String,
    /** Version of the crate. */
    val version: String,
    /** List of names of the credited authors. */
    val authors: List<String>,
)

/*
TODO do we really need the following classes?
  - I quite like that they are much simpler than the corresponding AST nodes (Input, Action, etc).
  - Also we have control over them. AST classes are generated by Xtend.
  - OTOH they're just a subset of the functionality of AST nodes for now. We could as well use them.
   But this would make this "IR" a weird mix between data classes and AST nodes.
  - Maybe the right data structures to use are the ReactorInstance, etc instead. But I feel like we're
   generating code not for the instance tree, but for now, just generic code for each reactor (like the C++
   generator)
 */


sealed class ReactorComponent {
    abstract val lfName: String

    companion object {
        /**
         * Convert an AST node for a reactor component to the corresponding dependency type.
         * Since there's no reasonable common supertype we use [Variable], but maybe we should
         * have another interface.
         */
        fun from(v: Variable): ReactorComponent? = when (v) {
            is Port   -> PortData(lfName = v.name, isInput = v.isInput, dataType = v.targetType)
            is Action -> ActionData(lfName = v.name, isLogical = v.isLogical)
            else      -> TODO("Unsupported: ${v.javaClass.simpleName} $v")
        }
    }
}

/**
 * @property dataType A piece of target code
 */
data class PortData(
    override val lfName: String,
    val isInput: Boolean,
    /** Rust data type of the code. */
    val dataType: TargetCode
) : ReactorComponent()

data class ActionData(
    override val lfName: String,
    val isLogical: Boolean
) : ReactorComponent()

/**
 * Produce model classes from the AST.
 */
object RustModelBuilder {

    /**
     * Given the input to the generator, produce the model classes.
     */
    fun makeGenerationInfo(reactors: List<Reactor>): GenerationInfo {
        val reactorsInfos = makeReactorInfos(reactors)
        // todo how do we pick the main reactor? it seems like super.doGenerate sets that field...
        val mainReactor = reactorsInfos.lastOrNull { it.isMain } ?: reactorsInfos.last()

        return GenerationInfo(
            crate = CrateInfo(
                name = mainReactor.lfName.camelToSnakeCase(),
                version = "1.0.0",
                authors = listOf(System.getProperty("user.name"))
            ),
            reactors = reactorsInfos,
            mainReactor = mainReactor,
            // Rust exec names are snake case, otherwise we get a cargo warning
            // https://github.com/rust-lang/rust/issues/45127
            executableName = mainReactor.lfName.camelToSnakeCase()
        )
    }

    private fun makeReactorInfos(reactors: List<Reactor>): List<ReactorInfo> =
        reactors.map { reactor ->
            val components = mutableMapOf<String, ReactorComponent>()
            for (component in reactor.allOutputs + reactor.allInputs + reactor.allActions) {
                val irObj = ReactorComponent.from(component) ?: continue
                components[irObj.lfName] = irObj
            }

            val reactions = reactor.reactions.map { n: Reaction ->
                val dependencies = (n.effects + n.sources).mapTo(LinkedHashSet()) { components[it.name]!! }
                ReactionInfo(
                    idx = n.indexInContainer,
                    depends = dependencies,
                    body = n.code.toText(),
                    isStartup = n.triggers.any { it.isStartup },
                    isShutdown = n.triggers.any { it.isShutdown }
                )
            }

            ReactorInfo(
                lfName = reactor.name,
                reactions = reactions,
                otherComponents = components.values.toList(),
                isMain = reactor.isMain,
                preambles = reactor.preambles.map { it.code.toText() },
                stateVars = reactor.stateVars.map {
                    StateVarInfo(
                        it.name,
                        it.targetType,
                        init = it.init.singleOrNull()?.toText()
                    )
                },
                nestedInstances = reactor.instantiations.map { it.toModel() }
            )
        }

    private fun Instantiation.toModel(): NestedReactorInstance {
        val params = ParamList.Positional(emptyList()) // fixme

        return NestedReactorInstance(
            lfName = this.name,
            params = params,
            reactorLfName = this.reactorClass.name
        )
    }
}


