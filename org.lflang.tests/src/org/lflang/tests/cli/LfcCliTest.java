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
import static org.lflang.tests.TestUtils.exists;
import static org.lflang.tests.TestUtils.isDirectory;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import org.lflang.LocalStrings;
import org.lflang.cli.Io;
import org.lflang.cli.Lfc;
import org.lflang.tests.cli.CliToolTestFixture.ExecutionResult;

/**
 * @author Clément Fournier
 */
public class LfcCliTest {

    LfcTestFixture lfcTester = new LfcTestFixture();

    static final String LF_PYTHON_FILE = """
        target Python
        main reactor {
            reaction(startup) {==}
        }
        """;


    @Test
    public void testHelpArg() {
        ExecutionResult result = lfcTester.run("--help", "--version");
        result.checkOk();
        result.checkNoErrorOutput();
        result.checkStdOut(containsString("usage: lfc"));
    }

    @Test
    public void testVersion() {
        ExecutionResult result = lfcTester.run("--version");
        result.checkOk();
        result.checkNoErrorOutput();
        result.checkStdOut(equalTo("lfc " + LocalStrings.VERSION + "\n"));
    }


    @Test
    public void testWrongCliArg() {
        ExecutionResult result = lfcTester.run("--notanargument", "File.lf");
        result.checkStdErr(containsString("Unrecognized option: --notanargument"));
        result.checkStdErr(containsString("fatal error"));
        result.checkFailed();
    }

    // todo is there a way to test this without invoking an actual
    // generator?
    @Test
    public void testGenInSrcDir(@TempDir Path tempDir) throws IOException {
        Files.createDirectories(tempDir.resolve("src"));
        Path srcFile = tempDir.resolve("src/File.lf");
        Files.writeString(srcFile, LF_PYTHON_FILE);

        ExecutionResult result = lfcTester.run(tempDir, "src/File.lf");

        result.checkOk();
        assertThat(tempDir.resolve("src-gen"), isDirectory());
        assertThat(tempDir.resolve("bin"), isDirectory());
        assertThat(tempDir.resolve("src-gen/File/File.py"), exists());
    }


    static class LfcTestFixture extends CliToolTestFixture {


        @Override
        protected void runCliProgram(Io io, String[] args) {
            Lfc.main(io, args);
        }
    }

}
