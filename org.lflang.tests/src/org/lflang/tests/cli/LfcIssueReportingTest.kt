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
package org.lflang.tests.cli

import org.junit.jupiter.api.Assertions.assertEquals
import org.junit.jupiter.api.Test
import org.lflang.LFRuntimeModule
import org.lflang.LFStandaloneSetup
import org.lflang.cli.AnsiColors
import org.lflang.cli.Io
import org.lflang.cli.LFStandaloneModule
import org.lflang.cli.Lfc
import org.lflang.cli.ReportingBackend
import org.opentest4j.AssertionFailedError
import java.io.ByteArrayOutputStream
import java.io.PrintStream
import java.nio.charset.StandardCharsets
import java.nio.file.Files
import java.nio.file.Path
import java.nio.file.Paths


class SpyPrintStream {
    private val baout = ByteArrayOutputStream()
    val ps = PrintStream(baout)

    override fun toString(): String = baout.toString(StandardCharsets.UTF_8)
}


class LfcIssueReportingTest {
    /*
        Note: when executing these tests in Intellij, I get the following error:

            java.lang.SecurityException: class "org.eclipse.core.runtime.IPath"'s
             signer information does not match signer information of other classes
             in the same package

        To fix this:
        - Go into File > Project Structure (CTRL+MAJ+S)
        - Open the "Modules" tab
        - Select the module org.lflang/lfc/test in the tree view
        - Open the "Dependencies" tab
        - Remove the dependency org.eclipse.platform:org.eclipse.equinox.common (it will have Provided scope)
     */


    @Test
    fun testSimpleWarning() {
        doTest(fileBaseName = "simpleWarning")
    }

    @Test
    fun testMultilineWarning() {
        doTest(fileBaseName = "multilineWarning")
    }

    @Test
    fun testTwoLineWarning() {
        doTest(fileBaseName = "twoLineWarning")
    }

    @Test
    fun testMultilineWarningTooBig() {
        doTest(fileBaseName = "multilineWarningTooBig")
    }

    @Test
    fun testIssue490() {
        doTest(fileBaseName = "issue490")
    }

    @Test
    fun testTabs() {
        doTest(fileBaseName = "tabs")
    }

    @Test
    fun testEmptyFile() {
        doTest(fileBaseName = "emptyFile")
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

        val io = Io(err = stderr.ps)
        val backend = ReportingBackend(io, AnsiColors(useColors).bold("lfc: "), AnsiColors(useColors), 2)
        val injector = LFStandaloneSetup(LFRuntimeModule(), LFStandaloneModule(backend, io))
            .createInjectorAndDoEMFRegistration()
        val main = injector.getInstance(Lfc::class.java)

        val packageName = loader.packageName.replace('.', '/')
        // relative to root of gradle project
        val basePath = "org.lflang.tests/resources/$packageName/"
        val lfFile = Paths.get("$basePath/$fileBaseName.lf")
        val expectedPath = Paths.get("$basePath/$fileBaseName.stderr")

        assert(Files.exists(lfFile)) { "Missing test file $lfFile" }

        // this side-effects on the ReportingBackend
        main.validateResource(main.getResource(lfFile))
        main.printErrorsIfAny()

        val actualOutput = stderr.toString().normalize(lfFile)

        if (!Files.exists(expectedPath)) {
            val lines = actualOutput.split("\n") // resplit on normalized delimiter
            Files.write(expectedPath, lines) // write using platform-specific delimiter, will be normalized by Git
            throw AssertionFailedError("Expected file $expectedPath does not exist, created it. Don't forget to `git add` it.")
        }
        val expected = Files.readString(expectedPath)

        assertEquals(expected.normalize(lfFile), actualOutput)
    }

    /**
     * Normalize [this] string to be platform independent.
     * @param lfFile Path to the test lf file
     */
    private fun String.normalize(lfFile: Path) =
        // normalize newlines and remove line-trailing whitespace
        trim().replace(Regex("\\s+\\R"), "\n")
            // Replace file path with placeholder. File path is not
            // rendered the same on linux and windows (forward/backward slash)
            .replace(lfFile.toString(), "%%%PATH.lf%%%")

}
