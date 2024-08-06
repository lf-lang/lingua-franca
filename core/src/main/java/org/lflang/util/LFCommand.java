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

package org.lflang.util;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.InputStream;
import java.io.PrintStream;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;
import org.eclipse.xtext.util.CancelIndicator;

/**
 * An abstraction for an external command
 *
 * <p>This is a wrapper around ProcessBuilder which allows for a more convenient usage in our code
 * base.
 */
public class LFCommand {

  /**
   * The period with which the cancel indicator is checked and the output and error streams are
   * forwarded.
   */
  private static final int PERIOD_MILLISECONDS = 200;

  /**
   * The maximum amount of time to wait for the forwarding of output and error streams to finish.
   */
  private static final int READ_TIMEOUT_MILLISECONDS = 1000;

  protected ProcessBuilder processBuilder;
  protected boolean didRun = false;
  protected ByteArrayOutputStream output = new ByteArrayOutputStream();
  protected ByteArrayOutputStream errors = new ByteArrayOutputStream();
  protected boolean quiet;

  /** Construct an LFCommand that executes the command carried by {@code pb}. */
  protected LFCommand(ProcessBuilder pb, boolean quiet) {
    this.processBuilder = pb;
    this.quiet = quiet;
  }

  /** Get the output collected during command execution */
  public String getOutput() {
    return output.toString();
  }

  /** Get the error output collected during command execution */
  public String getErrors() {
    return errors.toString();
  }

  /** Get this command's program and arguments. */
  public List<String> command() {
    return processBuilder.command();
  }

  /** Get this command's working directory. */
  public File directory() {
    return processBuilder.directory();
  }

  /** Get a String representation of the stored command */
  public String toString() {
    return String.join(" ", processBuilder.command());
  }

  /**
   * Collect as much output as possible from {@code in} without blocking, print it to {@code print}
   * if not quiet, and store it in {@code store}.
   */
  private void collectOutput(InputStream in, ByteArrayOutputStream store, PrintStream print) {
    byte[] buffer = new byte[64];
    int len;
    do {
      try {
        // This depends on in.available() being
        //  greater than zero if data is available
        //  (so that all data is collected)
        //  and upper-bounded by maximum number of
        //  bytes that can be read without blocking.
        //  Only the latter of these two conditions
        //  is guaranteed by the spec.
        len = in.read(buffer, 0, Math.min(in.available(), buffer.length));
        if (len > 0) {
          store.write(buffer, 0, len);
          if (!quiet) print.write(buffer, 0, len);
        }
      } catch (IOException e) {
        e.printStackTrace();
        break;
      }
    } while (len > 0); // Do not necessarily continue
    // to EOF (len == -1) because a blocking read
    // operation is hard to interrupt.
  }

  /**
   * Handle user cancellation if necessary, and handle any output from {@code process} otherwise.
   *
   * @param process a {@code Process}
   * @param cancelIndicator a flag indicating whether a cancellation of {@code process} is requested
   *     directly to stderr and stdout).
   */
  private void poll(Process process, CancelIndicator cancelIndicator) {
    if (cancelIndicator != null && cancelIndicator.isCanceled()) {
      process.descendants().forEach(ProcessHandle::destroyForcibly);
      process.destroyForcibly();
    } else {
      collectOutput(process.getInputStream(), output, System.out);
      collectOutput(process.getErrorStream(), errors, System.err);
    }
  }

  /**
   * Execute the command.
   *
   * <p>Executing a process directly with {@code processBuilder.start()} could lead to a deadlock as
   * the subprocess blocks when output or error buffers are full. This method ensures that output
   * and error messages are continuously read and forwards them to the system output and error
   * streams as well as to the output and error streams hold in this class.
   *
   * <p>If the current operation is cancelled (as indicated by <code>cancelIndicator</code>), the
   * subprocess is destroyed. Output and error streams until that point are still collected.
   *
   * @param cancelIndicator The indicator of whether the underlying process should be terminated.
   * @return the process' return code
   * @author Christian Menard
   */
  public int run(CancelIndicator cancelIndicator) {
    assert !didRun;
    didRun = true;

    // FIXME remove system out
    System.out.println("--- Current working directory: " + processBuilder.directory().toString());
    System.out.println("--- Executing command: " + String.join(" ", processBuilder.command()));

    final Process process = startProcess();
    if (process == null) return -1;

    ScheduledExecutorService poller = Executors.newSingleThreadScheduledExecutor();
    poller.scheduleAtFixedRate(
        () -> poll(process, cancelIndicator), 0, PERIOD_MILLISECONDS, TimeUnit.MILLISECONDS);

    try {
      final int returnCode = process.waitFor();
      poller.shutdown();
      poller.awaitTermination(READ_TIMEOUT_MILLISECONDS, TimeUnit.MILLISECONDS);
      // Finish collecting any remaining data
      poll(process, cancelIndicator);
      return returnCode;
    } catch (InterruptedException e) {
      e.printStackTrace();
      return -2;
    }
  }

  /**
   * Execute the command. Do not allow user cancellation.
   *
   * @return the process' return code
   */
  public int run() {
    return run(null);
  }

  /**
   * Add the given variables and their values to the command's environment.
   *
   * @param variables A map of variable names and their values
   */
  public void setEnvironmentVariables(Map<String, String> variables) {
    processBuilder.environment().putAll(variables);
  }

  /**
   * Add the given variable and its value to the command's environment.
   *
   * @param variableName name of the variable to add
   * @param value the variable's value
   */
  public void setEnvironmentVariable(String variableName, String value) {
    processBuilder.environment().put(variableName, value);
  }

  /**
   * Replace the given variable and its value in the command's environment.
   *
   * @param variableName name of the variable to add
   * @param value the variable's value
   */
  public void replaceEnvironmentVariable(String variableName, String value) {
    processBuilder.environment().remove(variableName);
    processBuilder.environment().put(variableName, value);
  }

  /** Require this to be quiet, overriding the verbosity specified at construction time. */
  public void setQuiet() {
    quiet = true;
  }

  /** Require this to be verbose, overriding the verbosity specified at construction time. */
  public void setVerbose() {
    quiet = false;
  }

  /**
   * Create a LFCommand instance from a given command and argument list in the current working
   * directory.
   *
   * @see #get(String, List, boolean, Path)
   */
  public static LFCommand get(final String cmd, final List<String> args) {
    return get(cmd, args, false, Paths.get(""));
  }

  /**
   * Create a LFCommand instance from a given command and argument list in the current working
   * directory.
   *
   * @see #get(String, List, boolean, Path)
   */
  public static LFCommand get(final String cmd, final List<String> args, boolean quiet) {
    return get(cmd, args, quiet, Paths.get(""));
  }

  /**
   * Create a LFCommand instance from a given command, an argument list and a directory.
   *
   * <p>This will check first if the command can actually be found and executed. If the command is
   * not found, null is returned. In order to find the command, different methods are applied in the
   * following order:
   *
   * <p>1. Check if the given command {@code cmd} is an executable file within {@code dir}. 2. If
   * the above fails 'which <cmd>' (or 'where <cmd>' on Windows) is executed to see if the command
   * is available on the PATH. 3. If both points above fail, a third attempt is started using bash
   * to indirectly execute the command (see below for an explanation).
   *
   * <p>A bit more context: If the command cannot be found directly, then a second attempt is made
   * using a Bash shell with the --login option, which sources the user's ~/.bash_profile,
   * ~/.bash_login, or ~/.bashrc (whichever is first found) before running the command. This helps
   * to ensure that the user's PATH variable is set according to their usual environment, assuming
   * that they use a bash shell.
   *
   * <p>More information: Unfortunately, at least on a Mac if you are running within Eclipse, the
   * PATH variable is extremely limited; supposedly, it is given by the default provided in
   * /etc/paths, but at least on my machine, it does not even include directories in that file for
   * some reason. One way to add a directory like /usr/local/bin to the path once-and-for-all is
   * this:
   *
   * <p>sudo launchctl config user path /usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin
   *
   * <p>But asking users to do that is not ideal. Hence, we try a more hack-y approach of just
   * trying to execute using a bash shell. Also note that while ProcessBuilder can be configured to
   * use custom environment variables, these variables do not affect the command that is to be
   * executed but merely the environment in which the command executes.
   *
   * @param cmd The command
   * @param args A list of arguments to pass to the command
   * @param quiet If true, the commands stdout and stderr will be suppressed
   * @param dir The directory in which the command should be executed
   * @return Returns an LFCommand if the given command could be found or null otherwise.
   */
  public static LFCommand get(final String cmd, final List<String> args, boolean quiet, Path dir) {
    assert cmd != null && args != null && dir != null;
    dir = dir.toAbsolutePath();

    // a list containing the command as first element and then all arguments
    List<String> cmdList = new ArrayList<>();
    cmdList.add(cmd);
    cmdList.addAll(args);

    ProcessBuilder builder = null;

    // First, see if the command is a local executable file
    final File cmdFile = dir.resolve(cmd).toFile();
    if (cmdFile.exists() && cmdFile.canExecute()) {
      builder = new ProcessBuilder(cmdList);
    } else if (findCommand(cmd) != null) {
      builder = new ProcessBuilder(cmdList);
    } else if (checkIfCommandIsExecutableWithBash(cmd, dir)) {
      builder =
          new ProcessBuilder(
              "bash", "--login", "-c", String.format("\"%s\"", String.join(" ", cmdList)));
    }

    if (builder != null) {
      builder.directory(dir.toFile());
      return new LFCommand(builder, quiet);
    }

    return null;
  }

  /**
   * Search for matches to the given command by following the PATH environment variable.
   *
   * @param command A command for which to search.
   * @return The file locations of matches to the given command.
   */
  private static List<File> findCommand(final String command) {
    final String whichCmd = System.getProperty("os.name").startsWith("Windows") ? "where" : "which";
    final ProcessBuilder whichBuilder = new ProcessBuilder(List.of(whichCmd, command));
    try {
      Process p = whichBuilder.start();
      if (p.waitFor() != 0) return null;
      return Arrays.stream(new String(p.getInputStream().readAllBytes()).split("\n"))
          .map(String::strip)
          .map(File::new)
          .filter(File::canExecute)
          .collect(Collectors.toList());
    } catch (InterruptedException | IOException e) {
      e.printStackTrace();
      return null;
    }
  }

  /**
   * Attempt to start the execution of this command.
   *
   * <p>First collect a list of paths where the executable might be found, then select an executable
   * that successfully executes from the list of paths. Return the {@code Process} instance that is
   * the result of a successful execution, or {@code null} if no successful execution happened.
   *
   * @return The {@code Process} that is started by this command, or {@code null} in case of
   *     failure.
   */
  private Process startProcess() {
    ArrayDeque<String> commands = new ArrayDeque<>();
    List<File> matchesOnPath = findCommand(processBuilder.command().get(0));
    if (matchesOnPath != null) {
      matchesOnPath.stream().map(File::toString).forEach(commands::addLast);
    }
    while (true) {
      try {
        return processBuilder.start();
      } catch (IOException e) {
        if (commands.isEmpty()) {
          e.printStackTrace();
          return null;
        }
      }
      processBuilder.command().set(0, commands.removeFirst());
    }
  }

  private static boolean checkIfCommandIsExecutableWithBash(final String command, final Path dir) {
    // check first if bash is installed
    if (findCommand("bash") == null) {
      return false;
    }

    // then try to find command with bash
    final ProcessBuilder bashBuilder =
        new ProcessBuilder(
            List.of("bash", "--login", "-c", String.format("\"which %s\"", command)));
    bashBuilder.directory(dir.toFile());
    try {
      int bashReturn = bashBuilder.start().waitFor();
      return bashReturn == 0;
    } catch (InterruptedException | IOException e) {
      e.printStackTrace();
      return false;
    }
  }
}
