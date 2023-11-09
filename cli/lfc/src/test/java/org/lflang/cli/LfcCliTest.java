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

package org.lflang.cli;

import static org.hamcrest.CoreMatchers.containsString;
import static org.hamcrest.CoreMatchers.equalTo;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.lflang.cli.TestUtils.TempDirBuilder.dirBuilder;
import static org.lflang.cli.TestUtils.TempDirChecker.dirChecker;
import static org.lflang.cli.TestUtils.isDirectory;
import static org.lflang.cli.TestUtils.isRegularFile;

import com.google.gson.JsonParser;
import com.google.inject.Injector;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.lflang.LocalStrings;
import org.lflang.cli.TestUtils.TempDirBuilder;
import org.lflang.generator.GeneratorArguments;
import org.lflang.target.property.BuildTypeProperty;
import org.lflang.target.property.CompilerProperty;
import org.lflang.target.property.LoggingProperty;
import org.lflang.target.property.NoCompileProperty;
import org.lflang.target.property.PrintStatisticsProperty;
import org.lflang.target.property.RuntimeVersionProperty;
import org.lflang.target.property.SchedulerProperty;
import org.lflang.target.property.SingleThreadedProperty;
import org.lflang.target.property.TargetProperty;
import org.lflang.target.property.type.BuildTypeType.BuildType;
import org.lflang.target.property.type.LoggingType.LogLevel;
import org.lflang.target.property.type.SchedulerType.Scheduler;

/**
 * @author Clément Fournier
 * @author Atharva Patil
 */
public class LfcCliTest {

  LfcTestFixture lfcTester = new LfcTestFixture();

  static final String LF_PYTHON_FILE =
      """
        target Python
        main reactor {
            reaction(startup) {==}
        }
        """;

  static final String JSON_STRING =
      """
        {
            "src": "src/File.lf",
            "out": "src",
            "properties": {
                "build-type": "Release",
                "clean": true,
                "compiler": "gcc",
                "external-runtime-path": "src",
                "federated": true,
                "logging": "info",
                "lint": true,
                "no-compile": true,
                "print-statistics": true,
                "quiet": true,
                "rti": "path/to/rti",
                "runtime-version": "rs",
                "scheduler": "GEDF_NP",
                "single-threaded": true
            }
        }
        """;

  @Test
  public void testHelpArg() {
    lfcTester
        .run("--help", "--version")
        .verify(
            result -> {
              result.checkOk();
              result.checkNoErrorOutput();
              result.checkStdOut(containsString("Usage:"));
              result.checkStdOut(containsString("lfc"));
            });
  }

  @Test
  public void testMutuallyExclusiveCliArgs() {
    lfcTester
        .run("File.lf", "--json", JSON_STRING)
        .verify(
            result -> {
              result.checkStdErr(containsString("are mutually exclusive (specify only one)"));
              result.checkFailed();
            });

    lfcTester
        .run("File.lf", "--json-file", "test.json")
        .verify(
            result -> {
              result.checkStdErr(containsString("are mutually exclusive (specify only one)"));
              result.checkFailed();
            });

    lfcTester
        .run("--json", JSON_STRING, "--json-file", "test.json")
        .verify(
            result -> {
              result.checkStdErr(containsString("are mutually exclusive (specify only one)"));
              result.checkFailed();
            });

    lfcTester
        .run("File.lf", "--single-threaded", "--workers", "1")
        .verify(
            result -> {
              result.checkStdErr(containsString("are mutually exclusive (specify only one)"));
              result.checkFailed();
            });
  }

  @Test
  public void testVersion() {
    lfcTester
        .run("--version")
        .verify(
            result -> {
              result.checkOk();
              result.checkNoErrorOutput();
              result.checkStdOut(equalTo("lfc " + LocalStrings.VERSION + System.lineSeparator()));
            });
  }

  @Test
  public void testWrongCliArg() {
    lfcTester
        .run("--notanargument", "File.lf")
        .verify(
            result -> {
              result.checkStdErr(containsString("Unknown option: '--notanargument'"));
              result.checkFailed();
            });
  }

  @Test
  public void testInvalidArgs(@TempDir Path tempDir) throws IOException {
    dirBuilder(tempDir).file("src/File.lf", LF_PYTHON_FILE);
    LfcOneShotTestFixture fixture = new LfcOneShotTestFixture();

    // Invalid src file.
    fixture
        .run(tempDir, "unknown.lf")
        .verify(
            result -> {
              result.checkStdErr(containsString("No such file or directory."));
              result.checkFailed();
            });

    // Invalid output path.
    fixture
        .run(tempDir, "--output-path", "unknown/output/path", "src/File.lf")
        .verify(
            result -> {
              result.checkStdErr(containsString("Output location does not exist."));
              result.checkFailed();
            });

    // Invalid build type.
    fixture
        .run(tempDir, "--build-type", "unknown-build-type", "src/File.lf")
        .verify(
            result -> {
              result.checkStdErr(containsString("Invalid build type."));
              result.checkFailed();
            });

    // Invalid logging level.
    fixture
        .run(tempDir, "--logging", "unknown_level", "src/File.lf")
        .verify(
            result -> {
              result.checkStdErr(containsString("Invalid log level."));
              result.checkFailed();
            });

    // Invalid RTI path.
    fixture
        .run(tempDir, "--rti", "unknown/rti/path", "src/File.lf")
        .verify(
            result -> {
              result.checkStdErr(containsString("Invalid RTI path."));
              result.checkFailed();
            });

    // Invalid scheduler.
    fixture
        .run(tempDir, "--scheduler", "unknown-scheduler", "src/File.lf")
        .verify(
            result -> {
              result.checkStdErr(containsString("Invalid scheduler."));
              result.checkFailed();
            });

    // Invalid workers.
    fixture
        .run(tempDir, "--workers", "notaninteger", "src/File.lf")
        .verify(
            result -> {
              result.checkStdErr(containsString("Invalid value for option '--workers'"));
              result.checkStdErr(containsString("is not an int"));
              result.checkFailed();
            });
  }

  @Test
  public void testGenInSrcDir(@TempDir Path tempDir) throws IOException {
    dirBuilder(tempDir).file("src/File.lf", LF_PYTHON_FILE);

    lfcTester
        .run(tempDir, "src/File.lf", "--no-compile")
        .verify(
            result -> {
              result.checkOk();
              dirChecker(tempDir)
                  .check("src-gen", isDirectory())
                  .check("bin", isDirectory())
                  .check("src-gen/File/File.py", isRegularFile());
            });
  }

  // Helper method for comparing argument values in tests testGeneratorArgs,
  // testGeneratorArgsJsonString and testGeneratorArgsJsonFile.
  public void verifyGeneratorArgs(Path tempDir, String[] args) {
    LfcOneShotTestFixture fixture = new LfcOneShotTestFixture();

    fixture
        .run(tempDir, args)
        .verify(
            result -> {
              // Don't validate execution because args are dummy args.
              var genArgs = fixture.lfc.getArgs();
              checkOverrideValue(genArgs, BuildTypeProperty.INSTANCE, BuildType.RELEASE);
              checkOverrideValue(genArgs, CompilerProperty.INSTANCE, "gcc");
              checkOverrideValue(genArgs, LoggingProperty.INSTANCE, LogLevel.INFO);
              checkOverrideValue(genArgs, NoCompileProperty.INSTANCE, true);
              checkOverrideValue(genArgs, PrintStatisticsProperty.INSTANCE, true);
              checkOverrideValue(genArgs, RuntimeVersionProperty.INSTANCE, "rs");
              checkOverrideValue(genArgs, SchedulerProperty.INSTANCE, Scheduler.GEDF_NP);
              checkOverrideValue(genArgs, SingleThreadedProperty.INSTANCE, true);

              assertEquals(true, genArgs.clean());
              assertEquals("src", Path.of(genArgs.externalRuntimeUri()).getFileName().toString());
              assertEquals(true, genArgs.lint());
              assertEquals(true, genArgs.quiet());
              assertEquals(
                  Path.of("path", "to", "rti"),
                  Path.of(new File("").getAbsolutePath()).relativize(Paths.get(genArgs.rti())));
            });
  }

  private void checkOverrideValue(
      GeneratorArguments args, TargetProperty<?, ?> property, Object expected) {
    var value =
        args.overrides().stream()
            .filter(a -> a.property().equals(property))
            .findFirst()
            .get()
            .value();
    assertEquals(expected, value);
  }

  public void verifyJsonGeneratorArgs(Path tempDir, String[] args) {
    LfcOneShotTestFixture fixture = new LfcOneShotTestFixture();

    fixture
        .run(tempDir, args)
        .verify(
            result -> {
              // Don't validate execution because args are dummy args.
              var genArgs = fixture.lfc.getArgs();
              assertEquals(
                  JsonParser.parseString(JSON_STRING).getAsJsonObject(), genArgs.jsonObject());
            });
  }

  @Test
  public void testGeneratorArgs(@TempDir Path tempDir) throws IOException {
    TempDirBuilder dir = dirBuilder(tempDir);
    dir.file("src/File.lf", LF_PYTHON_FILE);
    dir.mkdirs("path//to/rti");

    String[] args = {
      "src/File.lf",
      "--output-path",
      "src",
      "--build-type",
      "Release",
      "--clean",
      "--compiler",
      "gcc",
      "--external-runtime-path",
      "src",
      "--federated",
      "--logging",
      "info",
      "--lint",
      "--no-compile",
      "--print-statistics",
      "--quiet",
      "--rti",
      "path/to/rti",
      "--runtime-version",
      "rs",
      "--scheduler",
      "GEDF_NP",
      "--single-threaded"
    };
    verifyGeneratorArgs(tempDir, args);
  }

  @Test
  public void testGeneratorArgsJsonString(@TempDir Path tempDir) throws IOException {
    TempDirBuilder dir = dirBuilder(tempDir);
    dir.file("src/File.lf", LF_PYTHON_FILE);
    dir.mkdirs("path/to/rti");

    String[] args = {"--json", JSON_STRING};
    verifyJsonGeneratorArgs(tempDir, args);
  }

  @Test
  public void testGeneratorArgsJsonFile(@TempDir Path tempDir) throws IOException {
    TempDirBuilder dir = dirBuilder(tempDir);
    dir.file("src/File.lf", LF_PYTHON_FILE);
    dir.file("src/test.json", JSON_STRING);
    dir.mkdirs("path/to/rti");

    String[] args = {"--json-file", "src/test.json"};
    verifyJsonGeneratorArgs(tempDir, args);
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
