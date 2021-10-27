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
import org.eclipse.xtext.generator.IFileSystemAccess2
import org.eclipse.xtext.generator.IGeneratorContext
import org.lflang.FileConfig
import java.io.Closeable
import java.io.IOException
import java.nio.file.Files
import java.nio.file.Path
import kotlin.system.measureTimeMillis

class RustFileConfig(resource: Resource, fsa: IFileSystemAccess2, context: IGeneratorContext) :
    FileConfig(resource, fsa, context) {

    /**
     * Clean any artifacts produced by the C++ code generator.
     */
    @Throws(IOException::class)
    override fun doClean() {
        super.doClean()
        deleteDirectory(outPath.resolve("target"))
    }

    inline fun emit(p: Path, f: Emitter.() -> Unit) {
        //System.err.println("Generating file ${srcGenPath.relativize(p)}...")
        //val milliTime =
        measureTimeMillis {
            Emitter(p).use { it.f() }
        }
        //System.err.println("Done in $milliTime ms.")
    }

    inline fun emit(pathRelativeToOutDir: String, f: Emitter.() -> Unit): Unit = emit(srcGenPath.resolve(pathRelativeToOutDir), f)

}

/**
 * Builds the contents of a file. This is used with RAII, closing
 * the object writes to the file.
 */
class Emitter(
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
        Files.writeString(output, sb)
    }
}

