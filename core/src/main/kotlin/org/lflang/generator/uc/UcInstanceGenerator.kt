/*************
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
 ***************/

package org.lflang.generator.uc

import org.lflang.*
import org.lflang.generator.PrependOperator
import org.lflang.generator.uc.UcPortGenerator.Companion.codeType
import org.lflang.generator.uc.UcReactorGenerator.Companion.codeType
import org.lflang.lf.Instantiation
import org.lflang.lf.Port
import org.lflang.lf.Reactor
import org.lflang.validation.AttributeSpec

/** A code generator for reactor instances */
class UcInstanceGenerator(
    private val reactor: Reactor,
    private val fileConfig: UcFileConfig,
    private val messageReporter: MessageReporter
) {
    fun generateIncludes(): String =
        reactor.instantiations.map { fileConfig.getReactorHeaderPath(it.reactor) }
            .distinct()
            .joinToString(separator = "\n") { """#include "${it.toUnixString()}" """ }

    fun generateReactorStructFields() = reactor.instantiations.joinToString(prefix = "// Child reactor fields\n", separator = "\n", postfix = "\n") {
        "${it.reactor.codeType} ${it.name};"
    }

    fun generateReactorCtorCodes() = reactor.instantiations.joinToString(separator = "\n") {
        """|
           |${it.reactor.codeType}_ctor(&self->${it.name}, self->super.env, &self->super);
           |self->_children[child_idx++] = &self->${it.name}.super;
           |
        """.trimMargin()
    }
}
