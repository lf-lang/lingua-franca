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

import java.util.Locale

/*
    Model classes that serve as "intermediary representation" between the rust generator and emitter.
 */


data class GenerationInfo(
    val crate: CrateInfo,
    val runtime: RuntimeInfo,
    val reactors: List<ReactorInfo>,
    val mainReactor: ReactorInfo // it's also in the list
)

data class ReactorInfo(
    val lfName: String,
    val reactions: List<ReactionInfo>,
    val otherComponents: List<ReactorComponent>,
    val ctorParamTypes: List<String> = emptyList()
) {
    val modName = lfName.lowercase(Locale.ROOT)
    val structName get() = lfName
    val dispatcherName = "${structName}Dispatcher"
    val reactionIdName = "${structName}Reactions"
}

data class ReactionInfo(
    val idx: Int,
    /** The ID of the reaction in the reaction enum. */
    val rustId: String = "R$idx",
    /** The name of the worker function for this reaction. */
    val workerId: String = "react_$idx",
    /** Dependencies declared by the reaction, which are served to the worker function. */
    val depends: List<ReactorComponent>
)


data class CrateInfo(
    val name: String,
    val version: String,
    val authors: List<String>,
)

data class RuntimeInfo(
    val toml_spec: String,
    // options, etc
)


sealed class ReactorComponent {
    abstract val name: String
}

/**
 * @property dataType A piece of target code
 */
data class PortData(override val name: String, val input: Boolean, val dataType: String) : ReactorComponent()
data class ActionData(override val name: String, val isLogical: Boolean) : ReactorComponent()

