/*************
 * Copyright (c) 2019-2021, TU Dresden.

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

package org.lflang.generator.chisel

import org.eclipse.emf.common.util.EList
import org.eclipse.emf.ecore.resource.Resource
import org.lflang.generator.PrependOperator
import org.lflang.lf.Preamble
import org.lflang.lf.Reactor
import org.lflang.model
import org.lflang.scoping.LFGlobalScopeProvider
import org.lflang.toText
import org.lflang.toUnixString


class ChiselPreambleGenerator(
    private val reactor: Reactor,
) {
    /** A list of all preambles defined in the resource (file) */
    private val preambles: EList<Preamble> = reactor.preambles

    fun generatePreamble(): String {
        return with(PrependOperator) {
            """
                | // The following is copied from the user preamble
            ${" |"..preambles.joinToString(separator = "\n") { it.code.toText() }}
                | // End of user preamble.
            """.trimMargin()
        }
    }
}
