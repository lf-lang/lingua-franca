/*************
 * Copyright (c) 2019-2021, The University of California at Berkeley.

 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND 
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON 
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ***************/

package org.lflang.federated.launcher;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;

import org.lflang.ErrorReporter;
import org.lflang.TargetConfig;
import org.lflang.TargetProperty.ClockSyncMode;
import org.lflang.federated.generator.FedFileConfig;
import org.lflang.federated.generator.FederateInstance;

/**
 * Utility class that can be used to create a launcher for federated LF programs.
 * 
 * @author Edward A. Lee
 * @author Soroush Bateni
 */
public class FedLauncher {

    protected TargetConfig targetConfig;
    protected FedFileConfig fileConfig;
    protected ErrorReporter errorReporter;

    /**
     * @param targetConfig The current target configuration.
     * @param fileConfig The current file configuration.
     * @param errorReporter A error reporter for reporting any errors or warnings during the code generation
     */
    public FedLauncher(TargetConfig targetConfig, FedFileConfig fileConfig, ErrorReporter errorReporter) {
        this.targetConfig = targetConfig;
        this.fileConfig = fileConfig;
        this.errorReporter = errorReporter;
    }

    /**
     * Return the compile command for a federate.
     * 
     * @param federate The federate to compile.
     */
    protected String compileCommandForFederate(FederateInstance federate) {
        throw new UnsupportedOperationException("Don't know how to compile the federates.");
    }

    /**
     * Return the command that will execute a remote federate, assuming that the current
     * directory is the top-level project folder. This is used to create a launcher script
     * for federates.
     * 
     * @param federate The federate to execute.
     */
    protected String executeCommandForRemoteFederate(FederateInstance federate) {
        throw new UnsupportedOperationException("Don't know how to execute the federates.");
    }

    /**
     * Return the command that will execute a local federate, assuming that the current
     * directory is the top-level project folder. This is used to create a launcher script
     * for federates.
     * 
     * @param federate The federate to execute.
     */
    protected String executeCommandForLocalFederate(FedFileConfig fileConfig,
                                                    FederateInstance federate) {
        throw new UnsupportedOperationException("Don't know how to execute the federates.");
    }

    /**
     * Create the launcher shell scripts. This will create one or two files
     * in the output path (bin directory). The first has name equal to
     * the filename of the source file without the ".lf" extension.
     * This will be a shell script that launches the
     * RTI and the federates.  If, in addition, either the RTI or any
     * federate is mapped to a particular machine (anything other than
     * the default "localhost" or "0.0.0.0"), then this will generate
     * a shell script in the bin directory with name filename_distribute.sh
     * that copies the relevant source files to the remote host and compiles
     * them so that they are ready to execute using the launcher.
     * 
     * A precondition for this to work is that the user invoking this
     * code generator can log into the remote host without supplying
     * a password. Specifically, you have to have installed your
     * public key (typically found in ~/.ssh/id_rsa.pub) in
     * ~/.ssh/authorized_keys on the remote host. In addition, the
     * remote host must be running an ssh service.
     * On an Arch Linux system using systemd, for example, this means
     * running:
     * 
     *     sudo systemctl <start|enable> ssh.service
     * 
     * Enable means to always start the service at startup, whereas
     * start means to just start it this once.
     * 
     * On macOS, open System Preferences from the Apple menu and
     * click on the "Sharing" preference panel. Select the checkbox
     * next to "Remote Login" to enable it.
     * 
     * In addition, every host must have OpenSSL installed, with at least
     * version 1.1.1a.  You can check the version with
     *
     *     openssl version
     *
     * @param federates A list of federate instances in the federation
     * @param federationRTIProperties Contains relevant properties of the RTI.
     *  Can have values for 'host', 'dir', and 'user'
     */
    public void createLauncher(
        List<FederateInstance> federates,
        LinkedHashMap<String, Object> federationRTIProperties
    ) throws IOException {
        // NOTE: It might be good to use screen when invoking the RTI
        // or federates remotely, so you can detach and the process keeps running.
        // However, I was unable to get it working properly.
        // What this means is that the shell that invokes the launcher
        // needs to remain live for the duration of the federation.
        // If that shell is killed, the federation will die.
        // Hence, it is reasonable to launch the federation on a
        // machine that participates in the federation, for example,
        // on the machine that runs the RTI.  The command I tried
        // to get screen to work looks like this:
        // ssh -t «target» cd «path»; screen -S «filename»_«federate.name» -L bin/«filename»_«federate.name» 2>&1
        // var outPath = binGenPath
        StringBuilder shCode = new StringBuilder();
        StringBuilder distCode = new StringBuilder();
        shCode.append(getSetupCode() + "\n");
        String distHeader = getDistHeader(); 
        Object host = federationRTIProperties.get("host");
        Object target = host;

        Path path = Path.of(federationRTIProperties.get("dir") == null ? "LinguaFrancaRemote" : federationRTIProperties.get("dir").toString());

        Object user = federationRTIProperties.get("user");
        if (user != null) {
            target = user + "@" + host;
        }

        shCode.append("#### Host is " + host);
        
        // Launch the RTI in the foreground.
        if (host.equals("localhost") || host.equals("0.0.0.0")) {
            // FIXME: the paths below will not work on Windows
            shCode.append(getLaunchCode(getRtiCommand(federates, false)) + "\n");
        } else {
            // Start the RTI on the remote machine.
            // FIXME: Should $FEDERATION_ID be used to ensure unique directories, executables, on the remote host?
            // Copy the source code onto the remote machine and compile it there.
            if(distCode.length() == 0) distCode.append(distHeader + "\n");

            String logFileName = String.format("log/%s_RTI.log", fileConfig.name);

            // Launch the RTI on the remote machine using ssh and screen.
            // The -t argument to ssh creates a virtual terminal, which is needed by screen.
            // The -S gives the session a name.
            // The -L option turns on logging. Unfortunately, the -Logfile giving the log file name
            // is not standardized in screen. Logs go to screenlog.0 (or screenlog.n).
            // FIXME: Remote errors are not reported back via ssh from screen.
            // How to get them back to the local machine?
            // Perhaps use -c and generate a screen command file to control the logfile name,
            // but screen apparently doesn't write anything to the log file!
            //
            // The cryptic 2>&1 reroutes stderr to stdout so that both are returned.
            // The sleep at the end prevents screen from exiting before outgoing messages from
            // the federate have had time to go out to the RTI through the socket.

            shCode.append(getRemoteLaunchCode(host, target, logFileName, 
                    getRtiCommand(federates, true)) + "\n");
        }

        // Index used for storing pids of federates
        int federateIndex = 0;
        for (FederateInstance federate : federates) {
            if (federate.isRemote) {
                Path fedRelSrcGenPath = fileConfig.getOutPath().relativize(fileConfig.getSrcGenPath()).resolve(federate.name);
                if(distCode.length() == 0) distCode.append(distHeader + "\n");
                String logFileName = String.format("log/%s_%s.log", fileConfig.name, federate.name);
                String compileCommand = compileCommandForFederate(federate);
                // FIXME: Should $FEDERATION_ID be used to ensure unique directories, executables, on the remote host?
                distCode.append(getDistCode(path, federate, fedRelSrcGenPath, logFileName, fileConfig.getSrcGenPath(), compileCommand) + "\n");
                String executeCommand = executeCommandForRemoteFederate(federate);
                shCode.append(getFedRemoteLaunchCode(federate, path, logFileName, executeCommand, federateIndex++) + "\n");
            } else {
                String executeCommand = executeCommandForLocalFederate(fileConfig, federate);
                shCode.append(getFedLocalLaunchCode(federate, executeCommand, federateIndex++) + "\n");
            }
        }
        if (host.equals("localhost") || host.equals("0.0.0.0")) {
            // Local PID managements
            shCode.append("echo \"#### Bringing the RTI back to foreground so it can receive Control-C.\"" + "\n");
            shCode.append("fg %1" + "\n");
        }
        // Wait for launched processes to finish
        shCode.append(String.join("\n", 
            "echo \"RTI has exited. Wait for federates to exit.\"",
            "# Wait for launched processes to finish.",
            "# The errors are handled separately via trap.",
            "for pid in \"${pids[@]}\"",
            "do",
            "    wait $pid",
            "done",
            "echo \"All done.\""
        ) + "\n");

        // Create bin directory for the script.
        if (!Files.exists(fileConfig.binPath)) {
            Files.createDirectories(fileConfig.binPath);
        }

        System.out.println("##### Generating launcher for federation "
                               + " in directory "
                               + fileConfig.binPath);

        // Write the launcher file.
        // Delete file previously produced, if any.
        File file = fileConfig.binPath.resolve(fileConfig.name).toFile();
        if (file.exists()) {
            file.delete();
        }

        FileOutputStream fOut = new FileOutputStream(file);
        fOut.write(shCode.toString().getBytes());
        fOut.close();
        if (!file.setExecutable(true, false)) {
            errorReporter.reportWarning("Unable to make launcher script executable.");
        }

        // Write the distributor file.
        // Delete the file even if it does not get generated.
        file = fileConfig.binPath.resolve(fileConfig.name + "_distribute.sh").toFile();
        if (file.exists()) {
            file.delete();
        }
        if (distCode.length() > 0) {
            fOut = new FileOutputStream(file);
            fOut.write(distCode.toString().getBytes());
            fOut.close();
            if (!file.setExecutable(true, false)) {
                errorReporter.reportWarning("Unable to make distributor script executable.");
            }
        }
    }

    private String getSetupCode() {
        return String.join("\n", 
            "#!/bin/bash",
            "# Launcher for federated " + fileConfig.name + ".lf Lingua Franca program.",
            "# Uncomment to specify to behave as close as possible to the POSIX standard.",
            "# set -o posix",
            "",
            "# Enable job control",
            "set -m",
            "shopt -s huponexit",
            "",
            "# Set a trap to kill all background jobs on error or control-C",
            "# Use two distinct traps so we can see which signal causes this.",
            "cleanup() {",
            "    printf \"Killing federate %s.\\n\" ${pids[*]}",
            "    # The || true clause means this is not an error if kill fails.",
            "    kill ${pids[@]} || true",
            "    printf \"#### Killing RTI %s.\\n\" ${RTI}",
            "    kill ${RTI} || true",
            "    exit 1",
            "}",
            "cleanup_err() {",
            "    echo \"#### Received ERR signal on line $1. Invoking cleanup().\"",
            "    cleanup",
            "}",
            "cleanup_sigint() {",
            "    echo \"#### Received SIGINT signal on line $1. Invoking cleanup().\"",
            "    cleanup",
            "}",
            "",
            "trap 'cleanup_err $LINENO' ERR",
            "trap 'cleanup_sigint $LINENO' SIGINT",
            "",
            "# Create a random 48-byte text ID for this federation.",
            "# The likelihood of two federations having the same ID is 1/16,777,216 (1/2^24).",
            "FEDERATION_ID=`openssl rand -hex 24`",
            "echo \"Federate " + fileConfig.name + " in Federation ID '$FEDERATION_ID'\"",
            "# Launch the federates:"
        );
    }

    private String getDistHeader() {
        return String.join("\n", 
            "#!/bin/bash",
            "# Distributor for federated "+ fileConfig.name + ".lf Lingua Franca program.",
            "# Uncomment to specify to behave as close as possible to the POSIX standard.",
            "# set -o posix"
        );
    }

    private String getRtiCommand(List<FederateInstance> federates, boolean isRemote) {
        List<String> commands = new ArrayList<>();
        if (isRemote) {
            commands.add("RTI -i '${FEDERATION_ID}' \\");
        } else {
            commands.add("RTI -i ${FEDERATION_ID} \\");
        }
        if (targetConfig.auth) {
            commands.add("                        -a \\");
        }
        commands.addAll(List.of(
            "                        -n "+federates.size()+" \\",
            "                        -c "+targetConfig.clockSync.toString()+" \\"
        ));
        if (targetConfig.clockSync.equals(ClockSyncMode.ON)) {
            commands.add("period " + targetConfig.clockSyncOptions.period.toNanoSeconds()+" \\");
        }
        if (targetConfig.clockSync.equals(ClockSyncMode.ON) || 
                targetConfig.clockSync.equals(ClockSyncMode.INIT)) {
            commands.add("exchanges-per-interval "+targetConfig.clockSyncOptions.trials+" \\");
        }
        commands.add("&");
        return String.join("\n", commands);
    }

    private String getLaunchCode(String rtiLaunchCode) {
        return String.join("\n", 
            "echo \"#### Launching the runtime infrastructure (RTI).\"",
            "# First, check if the RTI is on the PATH",
            "if ! command -v RTI &> /dev/null",
            "then",
            "    echo \"RTI could not be found.\"",
            "    echo \"The source code can be obtained from https://github.com/lf-lang/reactor-c/tree/main/core/federated/RTI\"",
            "    exit",
            "fi                ",
            "# The RTI is started first to allow proper boot-up",
            "# before federates will try to connect.",
            "# The RTI will be brought back to foreground",
            "# to be responsive to user inputs after all federates",
            "# are launched.",
            rtiLaunchCode,
            "# Store the PID of the RTI",
            "RTI=$!",
            "# Wait for the RTI to boot up before",
            "# starting federates (this could be done by waiting for a specific output",
            "# from the RTI, but here we use sleep)",
            "sleep 1"
        );
    }

    private String getRemoteLaunchCode(Object host, Object target, String logFileName, String rtiLaunchString) {
        return String.join("\n", 
            "echo \"#### Launching the runtime infrastructure (RTI) on remote host " + host + ".\"",
            "# FIXME: Killing this ssh does not kill the remote process.",
            "# A double -t -t option to ssh forces creation of a virtual terminal, which",
            "# fixes the problem, but then the ssh command does not execute. The remote",
            "# federate does not start!",
            "ssh " + target + " 'mkdir -p log; \\",
            "    echo \"-------------- Federation ID: \"'$FEDERATION_ID' >> " + logFileName + "; \\",
            "    date >> " + logFileName + "; \\",
            "    echo \"Executing RTI: " + rtiLaunchString + "\" 2>&1 | tee -a " + logFileName + "; \\",
            "    # First, check if the RTI is on the PATH",
            "    if ! command -v RTI &> /dev/null",
            "    then",
            "        echo \"RTI could not be found.\"",
            "        echo \"The source code can be found in org.lflang/src/lib/core/federated/RTI\"",
            "        exit",
            "    fi",
            "    " + rtiLaunchString + " 2>&1 | tee -a " + logFileName + "' &",
            "# Store the PID of the channel to RTI",
            "RTI=$!",
            "# Wait for the RTI to boot up before",
            "# starting federates (this could be done by waiting for a specific output",
            "# from the RTI, but here we use sleep)",
            "sleep 5"
        );
    }

    private String getDistCode(
            Path remoteBase,
            FederateInstance federate, 
            Path remoteRelSrcGenPath,
            String logFileName,
            Path localAbsSrcGenPath,
            String compileCommand) {
        return String.join("\n",
                           "echo \"Making directory " + remoteBase
                               + " and subdirectories src-gen, bin, and log on host "
                               + getUserHost(federate.user, federate.host)
                               + "\"",
                           "# The >> syntax appends stdout to a file. The 2>&1 appends stderr to the same file.",
                           "ssh " + getUserHost(federate.user, federate.host)
                               + " '\\",
                           "    mkdir -p " + remoteBase.resolve(remoteRelSrcGenPath).resolve("core")
                               + " " + remoteBase.resolve("bin") + " "
                               + remoteBase + "/log; \\",
                           "    echo \"--------------\" >> " + remoteBase + "/"
                               + logFileName + "; \\",
                           "    date >> " + remoteBase + "/" + logFileName + ";",
                           "'",
                           "pushd " + localAbsSrcGenPath
                               + " > /dev/null",
                           "echo \"Copying source files to host "
                               + getUserHost(federate.user, federate.host)
                               + "\"",
                           "scp -r * "
                               + getUserHost(federate.user, federate.host) + ":"
                               + remoteBase.resolve(remoteRelSrcGenPath),
                           "popd > /dev/null",
                           "echo \"Compiling on host "
                               + getUserHost(federate.user, federate.host)
                               + " using: " + compileCommand + "\"",
                           "ssh " + getUserHost(federate.user, federate.host)
                               + " 'cd " + remoteBase + "; \\",
                           "    echo \"In " + remoteBase + " compiling with: "
                               + compileCommand + "\" >> " + logFileName
                               + " 2>&1; \\",
                           "    # Capture the output in the log file and stdout. \\",
                           "    " + compileCommand + " 2>&1 | tee -a "
                               + logFileName + ";' "
        );
    }

    private String getUserHost(Object user, Object host) {
        if (user == null) {
            return host.toString();
        }
        return user + "@" + host;
    }

    private String getFedRemoteLaunchCode(
            FederateInstance federate, 
            Object path,
            String logFileName,
            String executeCommand,
            int federateIndex
        ) {
        return String.join("\n", 
            "echo \"#### Launching the federate "+federate.name+" on host "+getUserHost(federate.user, federate.host)+"\"",
            "# FIXME: Killing this ssh does not kill the remote process.",
            "# A double -t -t option to ssh forces creation of a virtual terminal, which",
            "# fixes the problem, but then the ssh command does not execute. The remote",
            "# federate does not start!",
            "ssh "+getUserHost(federate.user, federate.host)+" '\\",
            "    cd "+path+"; \\",
            "    echo \"-------------- Federation ID: \"'$FEDERATION_ID' >> "+logFileName+"; \\",
            "    date >> "+logFileName+"; \\",
            "    echo \"In "+path+", executing: "+executeCommand+"\" 2>&1 | tee -a "+logFileName+"; \\",
            "    "+executeCommand+" 2>&1 | tee -a "+logFileName+"' &",
            "pids["+federateIndex+"]=$!"
        );
    }

    private String getFedLocalLaunchCode(FederateInstance federate, String executeCommand, int federateIndex) {
        return String.format(String.join("\n", 
            "echo \"#### Launching the federate %s.\"",
            "%s &",
            "pids[%s]=$!"
        ),
        federate.name,
        executeCommand,
        federateIndex);
    }
}
