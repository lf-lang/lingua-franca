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
package org.lflang.lfc.tests

import junit.framework.Assert.assertEquals
import junit.framework.AssertionFailedError
import org.junit.Test
import org.lflang.LFRuntimeModule
import org.lflang.LFStandaloneSetup
import org.lflang.lfc.*
import java.io.*
import java.nio.charset.StandardCharsets
import java.nio.file.Files
import java.nio.file.Paths


class SpyPrintStream {
    private val baout = ByteArrayOutputStream()
    val ps = PrintStream(baout)

    override fun toString(): String = baout.toString(StandardCharsets.UTF_8)
}


class LfIssueReportingTest {


    @Test
    fun testSimpleWarning() {
        doTest(fileBaseName = "simpleWarning")
    }

    @Test
    fun testMultilineWarning() {
        doTest(fileBaseName = "multilineWarning")
    }

    @Test
    fun testMultilineWarningTooBig() {
        doTest(fileBaseName = "multilineWarningTooBig")
    }

    @Test
    fun testTabs() {
        doTest(fileBaseName = "tabs")
    }

    @Test
    fun testColors() {
        doTest(fileBaseName = "colors", useColors = true)
    }

    /**
     * Looks for a file named [fileBaseName].lf to validate,
     * and its sibling [fileBaseName].stderr, which is the
     * expected output to stderr. Those files are in the test/resources directory
     */
    private fun doTest(fileBaseName: String, loader: Class<*> = this::class.java, useColors: Boolean = false) {


        val stderr = SpyPrintStream()

        val backend = ReportingBackend(Io(err = stderr.ps), AnsiColors(useColors), 2)
        val injector = LFStandaloneSetup(LFRuntimeModule(), LFStandaloneModule(backend))
            .createInjectorAndDoEMFRegistration()
        val main = injector.getInstance(Main::class.java)

        val packageName = loader.packageName.replace('.', '/')
        // relative to root of gradle project
        val lfFile = Paths.get("test/resources/$packageName/$fileBaseName.lf")
        val expectedPath = Paths.get("test/resources/$packageName/$fileBaseName.stderr")

        // this side-effects on the ReportingBackend
        main.getValidatedResource(lfFile)
        main.printErrorsIfAny()

        val actualOutput = stderr.toString()

        if (!Files.exists(expectedPath)) {
            Files.writeString(expectedPath, actualOutput)
            throw AssertionFailedError("Expected file $expectedPath does not exist, created it")
        }
        val expected = Files.readString(expectedPath)

        assertEquals(expected.normalize(), actualOutput.normalize())
    }

    private fun String.normalize() =
        // normalize newlines and remove line-trailing whitespace
        trim().replace(Regex("\\s+\\R"), "\n")

}
