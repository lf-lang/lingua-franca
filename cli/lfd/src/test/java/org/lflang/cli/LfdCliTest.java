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

import java.io.IOException;
import java.nio.file.FileVisitResult;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.SimpleFileVisitor;
import java.nio.file.attribute.BasicFileAttributes;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.lflang.LocalStrings;
import org.lflang.cli.CliToolTestFixture.ExecutionResult;

/**
 * @author Clément Fournier
 */
public class LfdCliTest {

  private static final String VALID_FILE =
      """
        target Python
        main reactor {
          reaction(startup) {==}
        }
        """;
  LfdTestFixture lfdTester = new LfdTestFixture();

  @Test
  public void testHelpArg() {
    ExecutionResult result = lfdTester.run("--help", "--version");
    result.checkOk();
    result.checkNoErrorOutput();
    result.checkStdOut(containsString("Usage:"));
    result.checkStdOut(containsString("lfd"));
  }

  @Test
  public void testVersion() {
    ExecutionResult result = lfdTester.run("--version");
    result.checkOk();
    result.checkNoErrorOutput();
    result.checkStdOut(equalTo("lfd " + LocalStrings.VERSION + System.lineSeparator()));
  }

  @Test
  public void testWrongCliArg() {
    ExecutionResult result = lfdTester.run("--notanargument", "File.lf");
    result.checkStdErr(containsString("Unknown option: '--notanargument'"));
    result.checkFailed();
  }

  @Test
  public void testValidFile(@TempDir Path tempDir) throws IOException {
    dirBuilder(tempDir).file("src/File.lf", VALID_FILE);
    ExecutionResult result = lfdTester.run(tempDir, "src/File.lf");
    result.checkOk();
    result.checkNoErrorOutput();
    Files.walkFileTree(
        tempDir,
        new SimpleFileVisitor<>() {
          @Override
          public FileVisitResult visitFile(Path file, BasicFileAttributes attrs) {
            System.out.println(file);
            return FileVisitResult.CONTINUE;
          }
        });
    assert tempDir.resolve("File.svg").toFile().exists();
  }

  @Test
  public void testInValidFile(@TempDir Path tempDir) throws IOException {
    ExecutionResult result = lfdTester.run(tempDir, ".");
    result.checkFailed();
  }

  static class LfdTestFixture extends CliToolTestFixture {
    @Override
    protected void runCliProgram(Io io, String[] args) {
      Lfd.main(io, args);
    }
  }
}
