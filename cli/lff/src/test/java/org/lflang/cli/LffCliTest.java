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
import static org.lflang.cli.TestUtils.TempDirBuilder.dirBuilder;
import static org.lflang.cli.TestUtils.TempDirChecker.dirChecker;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.lflang.LocalStrings;
import org.lflang.cli.CliToolTestFixture.ExecutionResult;

/**
 * @author Clément Fournier
 */
public class LffCliTest {

  private static final String FILE_BEFORE_REFORMAT =
      """
        target Python;
        main   reactor {
            reaction(startup) {=
               =}
        }
        """;
  private static final String FILE_AFTER_REFORMAT =
      """
        target Python

        main reactor {
          reaction(startup) {=  =}
        }
        """;

  private static final List<List<String>> TEST_CASES =
      List.of(
          List.of(
              """
                  target C
                  reactor Test { // this is a test
                  logical action a # this is an a
                  output humbug: int
                  reaction (a) -> /* moo */ humbug {=  // this is a humbug reaction
                    /* it reacts like this*/ react react
                  =}
                  }
                  """,
              """
                  target C

                  // this is a test
                  reactor Test {
                    logical action a  // this is an a
                    output humbug: int

                    /** moo */
                    // this is a humbug reaction
                    reaction(a) -> humbug {= /* it reacts like this*/ react react =}
                  }
                  """),
          List.of(
              """
                  target C
                  // Documentation
                   @icon("Variables.png")
                   reactor Variables {}
                  """,
              """
                  target C

                  // Documentation
                  @icon("Variables.png")
                  reactor Variables {
                  }
                  """),
          List.of(
              """
                  target C
                  reactor Filter(period: int = 0, b: double[](0, 0)) {}
                  main reactor {
                  az_f = new Filter(
                       period = 100,
                       b = (0.229019233988375, 0.421510777305010)
                   )
                   }
                   """,
              """
                  target C

                  reactor Filter(period: int = 0, b: double[] = {0, 0}) {
                  }

                  main reactor {
                    az_f = new Filter(period=100, b = {0.229019233988375, 0.421510777305010})
                  }
                  """),
          List.of(
              """
                  target Rust
                  reactor Snake {  // q
                  state grid: SnakeGrid ({= /* foo */ SnakeGrid::new(grid_side, &snake) =}); // note that this one borrows snake temporarily
                  state grid2: SnakeGrid ({= // baz
                  SnakeGrid::new(grid_side, &snake) =});
                  }
                  """,
              """
                  target Rust

                  // q
                  reactor Snake {
                    // note that this one borrows snake temporarily
                    state grid: SnakeGrid = {= /* foo */ SnakeGrid::new(grid_side, &snake) =}
                    // baz
                    state grid2: SnakeGrid = {= SnakeGrid::new(grid_side, &snake) =}
                  }
                  """),
          List.of(
              """
                  target Cpp

                  reactor ContextManager<Req, Resp, Ctx> {


                     \s
                  }

                  reactor MACService {
                    mul_cm = new ContextManager<loooooooooooooooooooooooooooooong, looooooooooooooong, loooooooooooooong>()
                  }

                  """,
              """
                  target Cpp

                  reactor ContextManager<Req, Resp, Ctx> {
                  }

                  reactor MACService {
                    mul_cm = new ContextManager<
                        loooooooooooooooooooooooooooooong,
                        looooooooooooooong,
                        loooooooooooooong>()
                  }
                  """));

  LffTestFixture lffTester = new LffTestFixture();

  @Test
  public void testHelpArg() {
    ExecutionResult result = lffTester.run("--help", "--version");
    result.checkOk();
    result.checkNoErrorOutput();
    result.checkStdOut(containsString("Usage:"));
    result.checkStdOut(containsString("lff"));
  }

  @Test
  public void testVersion() {
    ExecutionResult result = lffTester.run("--version");
    result.checkOk();
    result.checkNoErrorOutput();
    result.checkStdOut(equalTo("lff " + LocalStrings.VERSION + System.lineSeparator()));
  }

  @Test
  public void testWrongCliArg() {
    ExecutionResult result = lffTester.run("--notanargument", "File.lf");
    result.checkStdErr(containsString("Unknown option: '--notanargument'"));
    result.checkFailed();
  }

  @Test
  public void testFormatSingleFileInPlace(@TempDir Path tempDir) throws IOException {
    for (var pair : TEST_CASES) {
      var before = pair.get(0);
      var after = pair.get(1);
      dirBuilder(tempDir).file("src/File.lf", before);
      for (int i = 0; i < 2; i++) {
        ExecutionResult result = lffTester.run(tempDir, "src/File.lf");
        result.checkOk();
        dirChecker(tempDir).checkContentsOf("src/File.lf", equalTo(after));
      }
    }
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

    result.checkStdErr(containsString("Formatted src" + File.separator + "File.lf"));
    dirChecker(tempDir).checkContentsOf("src/File.lf", equalTo(FILE_AFTER_REFORMAT));
  }

  @Test
  public void testNoSuchFile(@TempDir Path tempDir) {
    ExecutionResult result = lffTester.run(tempDir, "-v", "nosuchdir");

    result.checkFailed();

    result.checkStdErr(
        containsString(tempDir.resolve("nosuchdir") + ": No such file or directory."));
  }

  @Test
  public void testOutputPathWithDirArg(@TempDir Path tempDir) throws IOException {
    dirBuilder(tempDir).file("src/a/File.lf", FILE_BEFORE_REFORMAT).mkdirs("out/");

    ExecutionResult result = lffTester.run(tempDir, "src", "--output-path", "out");

    result.checkOk();

    dirChecker(tempDir).checkContentsOf("out/a/File.lf", equalTo(FILE_AFTER_REFORMAT));
  }

  @Test
  public void testOutputPathWithFileArg(@TempDir Path tempDir) throws IOException {
    dirBuilder(tempDir).file("src/a/File.lf", FILE_BEFORE_REFORMAT).mkdirs("out/");

    ExecutionResult result = lffTester.run(tempDir, "src/a/File.lf", "--output-path", "out");

    result.checkOk();

    dirChecker(tempDir).checkContentsOf("out/File.lf", equalTo(FILE_AFTER_REFORMAT));
  }

  @Test
  public void testCheckAndDryRun(@TempDir Path tempDir) {
    lffTester.run(tempDir, "foo.lf", "--check", "--dry-run").checkFailed();
    lffTester.run(tempDir, "foo.lf", "-c", "-d").checkFailed();
  }

  @Test
  public void testCheck(@TempDir Path tempDir) throws IOException {
    dirBuilder(tempDir).file("src/a/File.lf", FILE_BEFORE_REFORMAT);
    dirBuilder(tempDir).file("src/b/File.lf", FILE_AFTER_REFORMAT);

    lffTester.run(tempDir, "src/a/File.lf", "--check").checkFailed();
    lffTester.run(tempDir, "src/a/File.lf", "-c").checkFailed();
    lffTester.run(tempDir, "src/b/File.lf", "--check").checkOk();
    lffTester.run(tempDir, "src/b/File.lf", "-c").checkOk();

    dirChecker(tempDir).checkContentsOf("src/a/File.lf", equalTo(FILE_BEFORE_REFORMAT));
    dirChecker(tempDir).checkContentsOf("src/b/File.lf", equalTo(FILE_AFTER_REFORMAT));
  }

  static class LffTestFixture extends CliToolTestFixture {

    @Override
    protected void runCliProgram(Io io, String[] args) {
      Lff.main(io, args);
    }
  }
}
