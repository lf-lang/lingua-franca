/*************
 * Copyright (c) 2019-2021 TU Dresden
 * Copyright (c) 2019-2021 UC Berkeley
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
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
 ***************/

package org.lflang.generator;

import static org.eclipse.lsp4j.DiagnosticSeverity.Error;
import static org.eclipse.lsp4j.DiagnosticSeverity.Warning;

import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Objects;
import org.eclipse.lsp4j.DiagnosticSeverity;
import org.lflang.FileConfig;
import org.lflang.MessageReporter;
import org.lflang.util.LFCommand;

/**
 * A factory class responsible for creating commands for use by the LF code generators.
 *
 * <p>In addition to the basic functionality of LFCommand, this class additionally ensures that
 * error messages (or optionally warnings) are shown when a command is not found and that certain
 * environment variables are set (see {@link #createCommand(String, List, Path, boolean)
 * createCommand}).
 */
public class GeneratorCommandFactory {

  protected final MessageReporter messageReporter;
  protected final FileConfig fileConfig;
  protected boolean quiet = false;

  public GeneratorCommandFactory(MessageReporter messageReporter, FileConfig fileConfig) {
    this.messageReporter = Objects.requireNonNull(messageReporter);
    this.fileConfig = Objects.requireNonNull(fileConfig);
  }

  /// enable quiet mode (command output is not printed)
  public void setQuiet() {
    quiet = true;
  }

  /// enable verbose mode (command output is printed)
  public void setVerbose() {
    quiet = false;
  }

  /**
   * Create a LFCommand instance from a given command and an argument list.
   *
   * <p>The command will be executed in the CWD and if the command cannot be found an error message
   * is shown.
   *
   * @param cmd the command to look up
   * @param args a list of arguments
   * @return an LFCommand object or null if the command could not be found
   * @see #createCommand(String, List, Path, boolean)
   */
  public LFCommand createCommand(String cmd, List<String> args) {
    return createCommand(cmd, args, null, true);
  }

  /**
   * Create a LFCommand instance from a given command, an argument list and a directory.
   *
   * @param cmd the command to look up
   * @param args a list of arguments
   * @param failOnError If true, an error is shown if the command cannot be found. Otherwise, only a
   *     warning is displayed.
   * @return an LFCommand object or null if the command could not be found
   * @see #createCommand(String, List, Path, boolean)
   */
  public LFCommand createCommand(String cmd, List<String> args, boolean failOnError) {
    return createCommand(cmd, args, null, failOnError);
  }

  /**
   * Create a LFCommand instance from a given command, an argument list and a directory.
   *
   * <p>The command will be executed in the CWD and if the command cannot be found an error message
   * is shown.
   *
   * @param cmd the command to look up
   * @param args a list of arguments
   * @param dir the directory to execute the command in. If null, this will default to the CWD
   * @return an LFCommand object or null if the command could not be found
   * @see #createCommand(String, List, Path, boolean)
   */
  public LFCommand createCommand(String cmd, List<String> args, Path dir) {
    return createCommand(cmd, args, dir, true);
  }

  /**
   * Create a LFCommand instance from a given command, an argument list and a directory.
   *
   * <p>This will check first if the command can actually be found and executed. If the command is
   * not found, null is returned. In addition, an error message will be shown if failOnError is
   * true. Otherwise, a warning will be displayed.
   *
   * @param cmd the command to look up
   * @param args a list of arguments
   * @param dir the directory to execute the command in. If null, this will default to the CWD
   * @param failOnError If true, an error is shown if the command cannot be found. Otherwise, only a
   *     warning is displayed.
   * @return an LFCommand object or null if the command could not be found
   * @see LFCommand#get(String, List, boolean, Path)
   */
  public LFCommand createCommand(String cmd, List<String> args, Path dir, boolean failOnError) {
    assert cmd != null && args != null;
    if (dir == null) {
      dir = Paths.get("");
    }
    LFCommand command = LFCommand.get(cmd, args, quiet, dir);
    if (command != null) {
      command.setEnvironmentVariable("LF_CURRENT_WORKING_DIRECTORY", dir.toString());
      command.setEnvironmentVariable("LF_SOURCE_DIRECTORY", fileConfig.srcPath.toString());
      command.setEnvironmentVariable("LF_PACKAGE_DIRECTORY", fileConfig.srcPkgPath.toString());
      command.setEnvironmentVariable(
          "LF_SOURCE_GEN_DIRECTORY", fileConfig.getSrcGenPath().toString());
      command.setEnvironmentVariable("LF_BIN_DIRECTORY", fileConfig.binPath.toString());
    } else {
      final String message =
          "The command "
              + cmd
              + " could not be found in the current working directory or in your PATH. "
              + "Make sure that your PATH variable includes the directory where "
              + cmd
              + " is installed. "
              + "You can set PATH in ~/.bash_profile on Linux or Mac.";

      DiagnosticSeverity severity = failOnError ? Error : Warning;
      messageReporter.nowhere().report(severity, message);
    }

    return command;
  }
}
