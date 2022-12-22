/*
 * Copyright (c) 2022, TU Dresden.
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

package org.lflang.generator

import java.util.*

/** Mediates access to the persisted Git revisions of the language runtime submodules. */
object LanguageRuntimeVersions {

    /** The committed version of the Rust runtime. */
    val rustRuntimeVersion: String by lazy {
        loadVersion("rs")
    }


    private fun loadVersion(languageId: String): String {
        val fileName = "lib/$languageId/runtime-version.properties"
        val stream = Thread.currentThread().contextClassLoader.getResourceAsStream(fileName)
            ?: throw IllegalStateException("Cannot find $fileName on classpath!")
        val props = Properties().apply { load(stream) }
        return props[languageId]?.toString()
            ?: throw IllegalStateException("Malformed properties file $fileName, expected a git commit as the $languageId entry")
    }


}
