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

import org.eclipse.emf.ecore.resource.Resource
import org.lflang.FileConfig
import org.lflang.TargetConfig
import org.lflang.camelToSnakeCase
import org.lflang.generator.CodeMap
import org.lflang.util.FileUtil
import java.io.Closeable
import java.io.IOException
import java.nio.file.Files
import java.nio.file.Path
import kotlin.system.measureTimeMillis

class RustFileConfig(resource: Resource, srcGenBasePath: Path, useHierarchicalBin: Boolean) :
    FileConfig(resource, srcGenBasePath, useHierarchicalBin) {

    /**
     * Clean any artifacts produced by the Rust code generator.
     */
    @Throws(IOException::class)
    override fun doClean() {
        super.doClean()
        FileUtil.deleteDirectory(outPath.resolve("target"))
    }

    override fun getExecutable(): Path {
        val localizedExecName = "${name.camelToSnakeCase()}$executableExtension"
        return binPath.resolve(localizedExecName)
    }

    override fun getExecutableExtension(): String {
        val isWindows = System.getProperty("os.name").lowercase().contains("win")
        return if (isWindows) {
            ".exe"
        } else {
            ""
        }
    }

    inline fun emit(codeMaps: MutableMap<Path, CodeMap>, p: Path, f: Emitter.() -> Unit) {
        measureTimeMillis {
            Emitter(codeMaps, p).use { it.f() }
        }
    }

    inline fun emit(codeMaps: MutableMap<Path, CodeMap>, pathRelativeToOutDir: String, f: Emitter.() -> Unit): Unit =
        emit(codeMaps, getSrcGenPath().resolve(pathRelativeToOutDir), f)
}

/**
 * Builds the contents of a file. This is used with RAII, closing
 * the object writes to the file.
 */
class Emitter(
    private val codeMaps: MutableMap<Path, CodeMap>,
    /** File to which this emitter should write. */
    private val output: Path,
) : Closeable {

    /** Accumulates the result. */
    private val sb = StringBuilder()
    private var indent: String = ""

    /**
     * Add the given string, which is taken as an entire line.
     * Indent is replaced with the contextual value.
     */
    operator fun plusAssign(line: String) {
        sb.append(line.replaceIndent(indent))
    }

    /**
     * Write the contents in an indented block
     */
    fun writeInBlock(header: String = "{", footer: String = "}", contents: Emitter.() -> Unit) {
        skipLines(1)
        sb.append(indent).append(header).appendLine()
        indent += "    "
        this.contents()
        sb.appendLine()
        indent = indent.removeSuffix("    ")
        sb.append(indent).append(footer).appendLine()
    }

    /**
     * Skip lines, the difference with just using [plusAssign]
     * is that no trailing whitespace is added.
     */
    fun skipLines(numLines: Int) {
        repeat(numLines) {
            sb.appendLine()
        }
    }

    override fun close() {
        Files.createDirectories(output.parent)
        val codeMap = CodeMap.fromGeneratedCode(sb.toString())
        codeMaps[output] = codeMap
        Files.writeString(output, codeMap.generatedCode)
    }
}
