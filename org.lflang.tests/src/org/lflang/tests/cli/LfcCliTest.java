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
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.lflang.tests.TestUtils.TempDirBuilder.dirBuilder;
import static org.lflang.tests.TestUtils.TempDirChecker.dirChecker;
import static org.lflang.tests.TestUtils.isDirectory;
import static org.lflang.tests.TestUtils.isRegularFile;

import java.io.IOException;
import java.lang.reflect.Method;
import java.nio.file.Path;
import java.util.Properties;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import org.lflang.LocalStrings;
import org.lflang.cli.Io;
import org.lflang.cli.Lfc;
import org.lflang.generator.LFGeneratorContext.BuildParm;

/**
 * @author Clément Fournier
 * @author Atharva Patil
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
        lfcTester.run("--help", "--version")
            .verify(result -> {
                result.checkOk();
                result.checkNoErrorOutput();
                result.checkStdOut(containsString("Usage: lfc"));
            });
    }

    @Test
    public void testVersion() {
        lfcTester.run("--version")
            .verify(result -> {
                result.checkOk();
                result.checkNoErrorOutput();
                result.checkStdOut(equalTo("lfc " + LocalStrings.VERSION + "\n"));
            });
    }


    @Test
    public void testWrongCliArg() {
        lfcTester.run("--notanargument", "File.lf")
            .verify(result -> {
                result.checkStdErr(containsString("Unknown option: '--notanargument'"));
                result.checkFailed();
            });
    }

    @Test
    public void testGenInSrcDir(@TempDir Path tempDir) throws IOException {
        dirBuilder(tempDir).file("src/File.lf", LF_PYTHON_FILE);

        lfcTester.run(tempDir, "src/File.lf", "--no-compile")
            .verify(result -> {
                result.checkOk();
                dirChecker(tempDir)
                    .check("src-gen", isDirectory())
                    .check("bin", isDirectory())
                    .check("src-gen/File/File.py", isRegularFile());
            });

    }

    @Test
    public void testTargetProperties(@TempDir Path tempDir)
            throws IOException {
        dirBuilder(tempDir).file("src/File.lf", LF_PYTHON_FILE);

        String[] args = {
            "src/File.lf",
            "--output-path", "src",
            "--build-type", "Release",
            "--clean",
            "--target-compiler", "gcc",
            "--external-runtime-path", "src",
            "--federated",
            "--logging", "4",
            "--lint",
            "--no-compile",
            "--quiet",
            "--rti", "-1",
            "--runtime-version", "rs",
            "--scheduler", "2",
            "--threading", "false",
            "--workers", "1",
        };

        lfcTester.runLfcObj(tempDir, args)
            .verify(result -> {
                result.checkOk();
                // Get the properties method.
                Method getPropsMethod =
                    Lfc.class.getDeclaredMethod("getTargetProperties");
                // Change the method's visibility to public for testing.
                getPropsMethod.setAccessible(true);
                Properties properties =
                    (Properties) getPropsMethod.invoke(result.lfcObj());
                assertEquals(
                        properties.getProperty(BuildParm.BUILD_TYPE.getKey()),
                        "Release");
                assertEquals(
                        properties.getProperty(BuildParm.CLEAN.getKey()),
                        "true");
                assertEquals(
                        properties.getProperty(
                            BuildParm.EXTERNAL_RUNTIME_PATH.getKey()),
                        "src");
                assertEquals(
                        properties.getProperty(BuildParm.LINT.getKey()),
                        "true");
                assertEquals(
                        properties.getProperty(BuildParm.LOGGING.getKey()),
                        "4");
                assertEquals(
                        properties.getProperty(
                            BuildParm.TARGET_COMPILER.getKey()),
                        "gcc");
                assertEquals(
                        properties.getProperty(BuildParm.QUIET.getKey()),
                        "true");
                assertEquals(
                        properties.getProperty(BuildParm.RTI.getKey()),
                        "-1");
                assertEquals(
                        properties.getProperty(
                            BuildParm.RUNTIME_VERSION.getKey()),
                        "rs");
                assertEquals(
                        properties.getProperty(BuildParm.THREADING.getKey()),
                        "false");
                assertEquals(
                        properties.getProperty(BuildParm.WORKERS.getKey()),
                        "1");
            });
    }


    static class LfcTestFixture extends CliToolTestFixture {
        @Override
        protected void runCliProgram(Io io, String[] args) {
            Lfc.main(io, args);
        }
    }

}
