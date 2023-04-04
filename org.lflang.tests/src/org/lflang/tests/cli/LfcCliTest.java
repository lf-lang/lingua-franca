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

import com.google.inject.Injector;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Properties;

import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import org.lflang.LocalStrings;
import org.lflang.cli.Io;
import org.lflang.cli.Lfc;
import org.lflang.generator.LFGeneratorContext.BuildParm;
import org.lflang.tests.TestUtils.TempDirBuilder;

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

    static final String JSON_STRING = """
        {
            "src": "src/File.lf",
            "out": "src",
            "properties": {
                "build-type": "Release",
                "clean": true,
                "target-compiler": "gcc",
                "external-runtime-path": "src",
                "federated": true,
                "logging": "info",
                "lint": true,
                "no-compile": true,
                "quiet": true,
                "rti": "path/to/rti",
                "runtime-version": "rs",
                "scheduler": "GEDF_NP",
                "threading": false,
                "workers": "1"
            }
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
    public void testMutuallyExclusiveCliArgs() {
        lfcTester.run("File.lf", "--json", JSON_STRING)
            .verify(result -> {
                result.checkStdErr(containsString(
                            "are mutually exclusive (specify only one)"));
                result.checkFailed();
            });

        lfcTester.run("File.lf", "--json-file", "test.json")
            .verify(result -> {
                result.checkStdErr(containsString(
                            "are mutually exclusive (specify only one)"));
                result.checkFailed();
            });

        lfcTester.run("--json", JSON_STRING, "--json-file", "test.json")
            .verify(result -> {
                result.checkStdErr(containsString(
                            "are mutually exclusive (specify only one)"));
                result.checkFailed();
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
    public void testInvalidArgs(@TempDir Path tempDir) throws IOException {
        dirBuilder(tempDir).file("src/File.lf", LF_PYTHON_FILE);
        LfcOneShotTestFixture fixture = new LfcOneShotTestFixture();

        // Invalid src file.
        fixture.run(tempDir, "unknown.lf")
            .verify(result -> {
                result.checkStdErr(containsString("No such file or directory."));
                result.checkFailed();
            });

        // Invalid output path.
        fixture.run(tempDir, "--output-path", "unknown/output/path", "src/File.lf")
            .verify(result -> {
                result.checkStdErr(containsString("Output location does not exist."));
                result.checkFailed();
            });

        // Invalid build type.
        fixture.run(tempDir, "--build-type", "unknown-build-type", "src/File.lf")
            .verify(result -> {
                result.checkStdErr(containsString("Invalid build type."));
                result.checkFailed();
            });

        // Invalid logging level.
        fixture.run(tempDir, "--logging", "unknown_level", "src/File.lf")
            .verify(result -> {
                result.checkStdErr(containsString("Invalid log level."));
                result.checkFailed();
            });

        // Invalid RTI path.
        fixture.run(tempDir, "--rti", "unknown/rti/path", "src/File.lf")
            .verify(result -> {
                result.checkStdErr(containsString("Invalid RTI path."));
                result.checkFailed();
            });

        // Invalid scheduler.
        fixture.run(tempDir, "--scheduler", "unknown-scheduler", "src/File.lf")
            .verify(result -> {
                result.checkStdErr(containsString("Invalid scheduler."));
                result.checkFailed();
            });

        // Invalid workers.
        fixture.run(tempDir, "--workers", "notaninteger", "src/File.lf")
            .verify(result -> {
                result.checkStdErr(containsString("Invalid value for option '--workers'"));
                result.checkStdErr(containsString("is not an int"));
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

    public void verifyGeneratorArgs(Path tempDir, String[] args) {
        LfcOneShotTestFixture fixture = new LfcOneShotTestFixture();

        fixture.run(tempDir, args)
            .verify(result -> {
                // Don't validate execution because args are dummy args.
                Properties properties = fixture.lfc.getGeneratorArgs();
                assertEquals(properties.getProperty(BuildParm.BUILD_TYPE.getKey()), "Release");
                assertEquals(properties.getProperty(BuildParm.CLEAN.getKey()), "true");
                assertEquals(properties.getProperty(BuildParm.TARGET_COMPILER.getKey()), "gcc");
                assertEquals(properties.getProperty(BuildParm.EXTERNAL_RUNTIME_PATH.getKey()), "src");
                assertEquals(properties.getProperty(BuildParm.LOGGING.getKey()), "info");
                assertEquals(properties.getProperty(BuildParm.LINT.getKey()), "true");
                assertEquals(properties.getProperty(BuildParm.NO_COMPILE.getKey()), "true");
                assertEquals(properties.getProperty(BuildParm.QUIET.getKey()), "true");
                assertEquals(properties.getProperty(BuildParm.RTI.getKey()), "path/to/rti");
                assertEquals(properties.getProperty(BuildParm.RUNTIME_VERSION.getKey()), "rs");
                assertEquals(properties.getProperty(BuildParm.SCHEDULER.getKey()), "GEDF_NP");
                assertEquals(properties.getProperty(BuildParm.THREADING.getKey()), "false");
                assertEquals(properties.getProperty(BuildParm.WORKERS.getKey()), "1");
            });
    }

    @Test
    public void testGeneratorArgs(@TempDir Path tempDir)
            throws IOException {
        TempDirBuilder dir = dirBuilder(tempDir);
        dir.file("src/File.lf", LF_PYTHON_FILE);
        dir.mkdirs("path//to/rti");

        String[] args = {
            "src/File.lf",
            "--output-path", "src",
            "--build-type", "Release",
            "--clean",
            "--target-compiler", "gcc",
            "--external-runtime-path", "src",
            "--federated",
            "--logging", "info",
            "--lint",
            "--no-compile",
            "--quiet",
            "--rti", "path/to/rti",
            "--runtime-version", "rs",
            "--scheduler", "GEDF_NP",
            "--threading", "false",
            "--workers", "1",
        };
        verifyGeneratorArgs(tempDir, args);
    }

    @Test
    public void testGeneratorArgsJsonString(@TempDir Path tempDir)
            throws IOException {
        TempDirBuilder dir = dirBuilder(tempDir);
        dir.file("src/File.lf", LF_PYTHON_FILE);
        dir.mkdirs("path//to/rti");

        String[] args = {"--json", JSON_STRING};
        verifyGeneratorArgs(tempDir, args);
    }

    @Test
    public void testGeneratorArgsJsonFile(@TempDir Path tempDir)
            throws IOException {
        TempDirBuilder dir = dirBuilder(tempDir);
        dir.file("src/File.lf", LF_PYTHON_FILE);
        dir.file("src/test.json", JSON_STRING);
        dir.mkdirs("path//to/rti");

        String[] args = {"--json-file", "src/test.json"};
        verifyGeneratorArgs(tempDir, args);
    }

    static class LfcTestFixture extends CliToolTestFixture {

        @Override
        protected void runCliProgram(Io io, String[] args) {
            Lfc.main(io, args);
        }
    }

    static class LfcOneShotTestFixture extends CliToolTestFixture {

        private Lfc lfc;

        @Override
        protected void runCliProgram(Io io, String[] args) {
            // Injector used to obtain Main instance.
            final Injector injector = Lfc.getInjector("lfc", io);
            // Main instance.
            this.lfc = injector.getInstance(Lfc.class);
            lfc.doExecute(io, args);
        }
    }
}
