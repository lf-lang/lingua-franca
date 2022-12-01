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

package org.lflang.tests.cli;

import static org.hamcrest.CoreMatchers.containsString;
import static org.hamcrest.CoreMatchers.equalTo;
import static org.hamcrest.MatcherAssert.assertThat;
import static org.lflang.tests.TestUtils.TempDirBuilder.dirBuilder;
import static org.lflang.tests.TestUtils.TempDirChecker.dirChecker;

import java.io.IOException;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import org.lflang.LocalStrings;
import org.lflang.tests.cli.CliToolTestFixture.ExecutionResult;
import org.lflang.cli.Io;
import org.lflang.cli.Lff;

/**
 * @author Clément Fournier
 */
public class LffCliTest {

    private static final String FILE_BEFORE_REFORMAT = """
        target Python;
        main   reactor {
            reaction(startup) {=
               =}
        }
        """;
    private static final String FILE_AFTER_REFORMAT = """
        target Python
            
        main reactor {
            reaction(startup) {=  =}
        }
        """;
    LffTestFixture lffTester = new LffTestFixture();


    @Test
    public void testHelpArg() {
        ExecutionResult result = lffTester.run("--help", "--version");
        result.checkOk();
        result.checkNoErrorOutput();
        result.checkStdOut(containsString("usage: lff"));
    }

    @Test
    public void testVersion() {
        ExecutionResult result = lffTester.run("--version");
        result.checkOk();
        result.checkNoErrorOutput();
        result.checkStdOut(equalTo("lff " + LocalStrings.VERSION + "\n"));
    }


    @Test
    public void testWrongCliArg() {
        ExecutionResult result = lffTester.run("--notanargument", "File.lf");
        result.checkStdErr(containsString("Unrecognized option: --notanargument"));
        result.checkStdErr(containsString("fatal error"));
        result.checkFailed();
    }

    @Test
    public void testFormatSingleFileInPlace(@TempDir Path tempDir) throws IOException {
        dirBuilder(tempDir).file("src/File.lf", FILE_BEFORE_REFORMAT);

        ExecutionResult result = lffTester.run(tempDir, "src/File.lf");

        result.checkOk();

        dirChecker(tempDir).checkContentsOf("src/File.lf", equalTo(FILE_AFTER_REFORMAT));
    }


    @Test
    public void testFormatDirectory(@TempDir Path tempDir) throws IOException {
        dirBuilder(tempDir).file("src/File.lf", FILE_BEFORE_REFORMAT);

        ExecutionResult result = lffTester.run(tempDir, "src");

        result.checkOk();

        dirChecker(tempDir).checkContentsOf("src/File.lf", equalTo(FILE_AFTER_REFORMAT));
    }


    @Test
    public void testFormatDirectoryVerbose(@TempDir Path tempDir) throws IOException {
        dirBuilder(tempDir).file("src/File.lf", FILE_BEFORE_REFORMAT);

        ExecutionResult result = lffTester.run(tempDir, "-v", "src");

        result.checkOk();

        result.checkStdOut(containsString("Formatted src/File.lf"));
        dirChecker(tempDir).checkContentsOf("src/File.lf", equalTo(FILE_AFTER_REFORMAT));
    }

    @Test
    public void testNoSuchFile(@TempDir Path tempDir) {
        ExecutionResult result = lffTester.run(tempDir, "-v", "nosuchdir");

        result.checkFailed();

        result.checkStdErr(containsString(
            tempDir.resolve("nosuchdir") + ": No such file or directory"));
    }


    @Test
    public void testOutputPathWithDirArg(@TempDir Path tempDir) throws IOException {
        dirBuilder(tempDir)
            .file("src/a/File.lf", FILE_BEFORE_REFORMAT)
            .mkdirs("out/");

        ExecutionResult result = lffTester.run(tempDir, "src", "--output-path", "out");

        result.checkOk();

        dirChecker(tempDir)
            .checkContentsOf("out/a/File.lf", equalTo(FILE_AFTER_REFORMAT));
    }

    @Test
    public void testOutputPathWithFileArg(@TempDir Path tempDir) throws IOException {
        dirBuilder(tempDir)
            .file("src/a/File.lf", FILE_BEFORE_REFORMAT)
            .mkdirs("out/");

        ExecutionResult result = lffTester.run(tempDir, "src/a/File.lf", "--output-path", "out");

        result.checkOk();

        dirChecker(tempDir)
            .checkContentsOf("out/File.lf", equalTo(FILE_AFTER_REFORMAT));
    }


    static class LffTestFixture extends CliToolTestFixture {


        @Override
        protected void runCliProgram(Io io, String[] args) {
            Lff.main(io, args);
        }
    }

}
