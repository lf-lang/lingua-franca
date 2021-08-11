/*************
 Copyright (c) 2019-2021 TU Dresden
 Copyright (c) 2019-2021 UC Berkeley

 Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

 1. Redistributions of source code must retain the above copyright notice,
 this list of conditions and the following disclaimer.

 2. Redistributions in binary form must reproduce the above copyright notice,
 this list of conditions and the following disclaimer in the documentation
 and/or other materials provided with the distribution.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.util;

import java.io.ByteArrayOutputStream;
import java.io.File;
import java.io.IOException;
import java.io.OutputStream;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * An abstraction for an external command
 * <p>
 * This is a wrapper around ProcessBuilder which allows for a more convenient usage in our code base.
 */
public class LFCommand {

    protected ProcessBuilder processBuilder;
    protected boolean didRun = false;
    protected ByteArrayOutputStream output = new ByteArrayOutputStream();
    protected ByteArrayOutputStream errors = new ByteArrayOutputStream();


    /**
     * Constructor
     */
    protected LFCommand(ProcessBuilder pb) { processBuilder = pb; }


    /**
     * Get the output collected during command execution
     */
    public OutputStream getOutput() { return output; }


    /**
     * Get the error output collected during command execution
     */
    public OutputStream getErrors() { return errors; }


    /**
     * Get a String representation of the stored command
     */
    public String toString() { return String.join(" ", processBuilder.command()); }


    /**
     * A runnable that collects the output from a running process, prints it and stores it in the output stream
     */
    private class OutputCollector implements Runnable {

        private final Process process;


        OutputCollector(Process process) {
            this.process = process;
        }


        @Override
        public void run() {
            byte[] buffer = new byte[64];
            int len;
            do {
                try {
                    len = process.getInputStream().read(buffer);
                    if (len > 0) {
                        output.write(buffer, 0, len);
                        System.out.write(buffer, 0, len);
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                    break;
                }
            } while (len != -1);
        }
    }

    /**
     * A runnable that collects the error output from a running process, prints it and stores it in the output stream
     */
    private class ErrorCollector implements Runnable {

        private final Process process;


        ErrorCollector(Process process) {
            this.process = process;
        }


        @Override
        public void run() {
            byte[] buffer = new byte[64];
            int len;
            do {
                try {
                    len = process.getErrorStream().read(buffer);
                    if (len > 0) {
                        errors.write(buffer, 0, len);
                        System.err.write(buffer, 0, len);
                    }
                } catch (IOException e) {
                    e.printStackTrace();
                    break;
                }
            } while (len != -1);
        }
    }


    /**
     * Execute the command while forwarding output and error streams.
     * <p>
     * Executing a process directly with `processBuiler.start()` could
     * lead to a deadlock as the subprocess blocks when output or error
     * buffers are full. This method ensures that output and error messages
     * are continuously read and forwards them to the system output and
     * error streams as well as to the output and error streams hold in
     * this class.
     *
     * @return the process' return code
     * @author {Christian Menard <christian.menard@tu-dresden.de}
     */
    public int run() {
        assert !didRun;
        didRun = true;

        System.out.println("--- Current working directory: " + processBuilder.directory().toString());
        System.out.println("--- Executing command: " + String.join(" ", processBuilder.command()));

        final Process process;
        try {
            process = processBuilder.start();
        } catch (IOException e) {
            e.printStackTrace();
            return -1;
        }

        final Thread collectOutputThread = new Thread(new OutputCollector(process));
        final Thread collectErrorsThread = new Thread(new ErrorCollector(process));

        collectOutputThread.start();
        collectErrorsThread.start();

        try {
            final int returnCode = process.waitFor();
            collectOutputThread.join();
            collectErrorsThread.join();
            return returnCode;
        } catch (InterruptedException e) {
            e.printStackTrace();
            return -2;
        }
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
     * @param value        the variable's value
     */
    public void setEnvironmentVariable(String variableName, String value) {
        processBuilder.environment().put(variableName, value);
    }
    
    /**
     * Replace the given variable and its value in the command's environment.
     * 
     * @param variableName name of the variable to add
     * @param value        the variable's value
     */
    public void replaceEnvironmentVariable(String variableName, String value) {
        processBuilder.environment().remove(variableName);
        processBuilder.environment().put(variableName, value);
    }

    /**
     * Create a LFCommand instance from a given command and argument list in the current working directory.
     *
     * @see #get(String, List, Path)
     */
    public static LFCommand get(final String cmd, final List<String> args) {
        return get(cmd, args, Paths.get(""));
    }


    /**
     * Create a LFCommand instance from a given command, an argument list and a directory.
     * <p>
     * This will check first if the command can actually be found and executed. If the command is not found, null is
     * returned. In order to find the command, different methods are applied in the following order:
     * <p>
     * 1. Check if the given command `cmd` is an executable file within `dir`.
     * 2. If the above fails 'which <cmd>' (or 'where <cmd>' on Windows) is executed to see if the command is available
     * on the PATH.
     * 3. If both points above fail, a third attempt is started using bash to indirectly execute the command (see below
     * for an explanation).
     * <p>
     * A bit more context:
     * If the command cannot be found directly, then a second attempt is made using a Bash shell with the --login
     * option, which sources the user's ~/.bash_profile, ~/.bash_login, or ~/.bashrc (whichever is first found) before
     * running the command. This helps to ensure that the user's PATH variable is set according to their usual
     * environment, assuming that they use a bash shell.
     * <p>
     * More information: Unfortunately, at least on a Mac if you are running within Eclipse, the PATH variable is
     * extremely limited; supposedly, it is given by the default provided in /etc/paths, but at least on my machine, it
     * does not even include directories in that file for some reason. One way to add a directory like /usr/local/bin
     * to
     * the path once-and-for-all is this:
     * <p>
     * sudo launchctl config user path /usr/bin:/bin:/usr/sbin:/sbin:/usr/local/bin
     * <p>
     * But asking users to do that is not ideal. Hence, we try a more hack-y approach of just trying to execute using a
     * bash shell. Also note that while ProcessBuilder can configured to use custom environment variables, these
     * variables do not affect the command that is to be executed but merely the environment in which the command
     * executes.
     *
     * @param cmd  The command
     * @param args A list of arguments to pass to the command
     * @param dir  The directory in which the command should be executed
     * @return Returns an LFCommand if the given command could be found or null otherwise.
     */
    public static LFCommand get(final String cmd, final List<String> args, Path dir) {
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
        } else if (checkIfCommandIsOnPath(cmd, dir)) {
            builder = new ProcessBuilder(cmdList);
        } else if (checkIfCommandIsExecutableWithBash(cmd, dir)) {
            builder = new ProcessBuilder("bash", "--login", "-c", String.join(" ", cmdList));
        }

        if (builder != null) {
            builder.directory(dir.toFile());
            return new LFCommand(builder);
        }

        return null;
    }


    private static boolean checkIfCommandIsOnPath(final String command, final Path dir) {
        final String whichCmd = System.getProperty("os.name").startsWith("Windows") ? "where" : "which";
        final ProcessBuilder whichBuilder = new ProcessBuilder(List.of(whichCmd, command));
        whichBuilder.directory(dir.toFile());
        try {
            int whichReturn = whichBuilder.start().waitFor();
            return whichReturn == 0;
        } catch (InterruptedException | IOException e) {
            e.printStackTrace();
            return false;
        }
    }


    private static boolean checkIfCommandIsExecutableWithBash(final String command, final Path dir) {
        // check first if bash is installed
        if (!checkIfCommandIsOnPath("bash", dir)) {
            return false;
        }

        // then try to find command with bash
        final ProcessBuilder bashBuilder = new ProcessBuilder(List.of("bash", "--login", "-c", "which " + command));
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
