package org.lflang.federated.launcher;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.lflang.MessageReporter;
import org.lflang.federated.generator.FederateInstance;
import org.lflang.federated.generator.FederationFileConfig;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.AuthProperty;
import org.lflang.target.property.ClockSyncModeProperty;
import org.lflang.target.property.ClockSyncOptionsProperty;
import org.lflang.target.property.DNETProperty;
import org.lflang.target.property.TracingProperty;
import org.lflang.target.property.type.ClockSyncModeType.ClockSyncMode;

/**
 * Utility class that can be used to create a launcher for federated LF programs.
 *
 * @author Edward A. Lee
 * @author Soroush Bateni
 * @ingroup Federated
 */
public class FedLauncherGenerator {
  protected TargetConfig targetConfig;
  protected FederationFileConfig fileConfig;
  protected MessageReporter messageReporter;

  /**
   * @param targetConfig The current target configuration.
   * @param fileConfig The current file configuration.
   * @param messageReporter A error reporter for reporting any errors or warnings during the code
   *     generation
   */
  public FedLauncherGenerator(
      TargetConfig targetConfig, FederationFileConfig fileConfig, MessageReporter messageReporter) {
    this.targetConfig = targetConfig;
    this.fileConfig = fileConfig;
    this.messageReporter = messageReporter;
  }

  /**
   * Create the launcher shell scripts. This will create one or two files in the output path (bin
   * directory). The first has name equal to the filename of the source file without the ".lf"
   * extension. This will be a shell script that launches the RTI and the federates. If, in
   * addition, either the RTI or any federate is mapped to a particular machine (anything other than
   * the default "localhost" or "0.0.0.0"), then this will generate a shell script in the bin
   * directory with name filename_distribute.sh that copies the relevant source files to the remote
   * host and compiles them so that they are ready to execute using the launcher.
   *
   * <p>A precondition for this to work is that the user invoking this code generator can log into
   * the remote host without supplying a password. Specifically, you have to have installed your
   * public key (typically found in ~/.ssh/id_rsa.pub) in ~/.ssh/authorized_keys on the remote host.
   * In addition, the remote host must be running an ssh service. On an Arch Linux system using
   * systemd, for example, this means running:
   *
   * <p>sudo systemctl <start|enable> ssh.service
   *
   * <p>Enable means to always start the service at startup, whereas start means to just start it
   * this once.
   *
   * <p>On macOS, open System Preferences from the Apple menu and click on the "Sharing" preference
   * panel. Select the checkbox next to "Remote Login" to enable it.
   *
   * <p>In addition, every host must have OpenSSL installed, with at least version 1.1.1a. You can
   * check the version with
   *
   * <p>openssl version
   *
   * @param federates A list of federate instances in the federation
   * @param rtiConfig Can have values for 'host', 'dir', and 'user'
   */
  public void doGenerate(List<FederateInstance> federates, RtiConfig rtiConfig) {
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
    // ssh -t «target» cd «path»; screen -S «filename»_«federate.name» -L
    // bin/«filename»_«federate.name» 2>&1
    // var outPath = binGenPath
    StringBuilder shCode = new StringBuilder();
    StringBuilder distCode = new StringBuilder();
    shCode.append(getSetupCode()).append("\n");
    String distHeader = getDistHeader();
    String host = rtiConfig.getHost();
    String target = host;

    String user = rtiConfig.getUser();
    if (user != null) {
      target = user + "@" + host;
    }

    shCode.append("#### Host is ").append(host).append("\n");

    // Launch the RTI in the foreground.
    if (host.equals("localhost") || host.equals("0.0.0.0")) {
      // FIXME: the paths below will not work on Windows
      shCode
          .append(
              getLaunchCode(getRtiCommand(fileConfig.getRtiBinPath().toString(), federates, false)))
          .append("\n");
    } else {
      // Copy the RTI to the remote machine and compile it there.
      // FIXME: Should $FEDERATION_ID be used to ensure unique directories, executables, on the
      // remote host?
      // Copy the source code onto the remote machine and compile it there.
      if (distCode.length() == 0) distCode.append(distHeader).append("\n");

      distCode
          .append(
              getDistCode(
                  rtiConfig.getDirectory(), "RTI", rtiConfig.getUser(), rtiConfig.getHost()))
          .append("\n");

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

      shCode
          .append(
              getRemoteLaunchCode(
                  host,
                  target,
                  logFileName,
                  getRtiCommand(rtiConfig.getRtiBinPath(fileConfig), federates, true)))
          .append("\n");
    }

    // Index used for storing pids of federates
    int federateIndex = 0;
    for (FederateInstance federate : federates) {
      var buildConfig = getBuildConfig(federate, fileConfig, messageReporter);
      if (federate.isRemote) {
        Path fedRelSrcGenPath =
            fileConfig.getOutPath().relativize(fileConfig.getSrcGenPath()).resolve(federate.name);
        if (distCode.isEmpty()) distCode.append(distHeader).append("\n");
        String logFileName = String.format("log/%s_%s.log", fileConfig.name, federate.name);
        distCode
            .append(
                getDistCode(rtiConfig.getDirectory(), federate.name, federate.user, federate.host))
            .append("\n");
        shCode
            .append(getFedRemoteLaunchCode(rtiConfig.getDirectory(), federate, federateIndex++))
            .append("\n");
      } else {
        String executeCommand = buildConfig.localExecuteCommand();
        shCode
            .append(getFedLocalLaunchCode(federate, executeCommand, federateIndex++))
            .append("\n");
      }
    }
    if (host.equals("localhost") || host.equals("0.0.0.0")) {
      // Local PID managements
      shCode.append(
          "echo \"#### Bringing the RTI back to foreground so it can receive Control-C.\"" + "\n");
      shCode.append("fg %1" + "\n");
    }
    // Wait for launched processes to finish
    shCode
        .append(
            String.join(
                "\n",
                "echo \"RTI has exited. Wait for federates to exit.\"",
                "# Wait for launched processes to finish.",
                "# The errors are handled separately via trap.",
                "for pid in \"${pids[@]}\"",
                "do",
                "    wait $pid || exit $?",
                "done",
                "echo \"All done.\"",
                "EXITED_SUCCESSFULLY=true"))
        .append("\n");

    // Create bin directory for the script.
    if (!Files.exists(fileConfig.binPath)) {
      try {
        Files.createDirectories(fileConfig.binPath);
      } catch (IOException e) {
        messageReporter.nowhere().error("Unable to create directory: " + fileConfig.binPath);
      }
    }

    // Write the launcher file.
    File file = fileConfig.binPath.resolve(fileConfig.name).toFile();
    messageReporter.nowhere().info("Script for launching the federation: " + file);

    // Delete file previously produced, if any.
    if (file.exists()) {
      if (!file.delete())
        messageReporter
            .nowhere()
            .error("Failed to delete existing federated launch script \"" + file + "\"");
    }

    FileOutputStream fOut = null;
    try {
      fOut = new FileOutputStream(file);
    } catch (FileNotFoundException e) {
      messageReporter.nowhere().error("Unable to find file: " + file);
    }
    if (fOut != null) {
      try {
        fOut.write(shCode.toString().getBytes());
        fOut.close();
      } catch (IOException e) {
        messageReporter.nowhere().error("Unable to write to file: " + file);
      }
    }

    if (!file.setExecutable(true, false)) {
      messageReporter.nowhere().warning("Unable to make launcher script executable.");
    }

    // Write the distributor file.
    // Delete the file even if it does not get generated.
    file = fileConfig.binPath.resolve(fileConfig.name + "_distribute.sh").toFile();
    if (file.exists()) {
      if (!file.delete())
        messageReporter
            .nowhere()
            .error("Failed to delete existing federated distributor script \"" + file + "\"");
    }
    if (distCode.length() > 0) {
      try {
        fOut = new FileOutputStream(file);
        fOut.write(distCode.toString().getBytes());
        fOut.close();
        if (!file.setExecutable(true, false)) {
          messageReporter.nowhere().warning("Unable to make file executable: " + file);
        }
      } catch (FileNotFoundException e) {
        messageReporter.nowhere().error("Unable to find file: " + file);
      } catch (IOException e) {
        messageReporter.nowhere().error("Unable to write to file " + file);
      }
    }
  }

  private String getSetupCode() {
    return String.join(
        "\n",
        "#!/bin/bash -l",
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
        "    if [ \"$EXITED_SUCCESSFULLY\" = true ] ; then",
        "        exit 0",
        "    else",
        "        printf \"Killing federate %s.\\n\" ${pids[*]}",
        "        # The || true clause means this is not an error if kill fails.",
        "        kill ${pids[@]} || true",
        "        printf \"#### Killing RTI %s.\\n\" ${RTI}",
        "        kill ${RTI} || true",
        "        exit 1",
        "    fi",
        "}",
        "",
        "trap 'cleanup; exit' EXIT",
        "",
        "# Create a random 48-byte text ID for this federation.",
        "# The likelihood of two federations having the same ID is 1/16,777,216 (1/2^24).",
        "FEDERATION_ID=`openssl rand -hex 24`",
        "echo \"Federate " + fileConfig.name + " in Federation ID '$FEDERATION_ID'\"",
        "# Launch the federates:");
  }

  private String getDistHeader() {
    return String.join(
        "\n",
        "#!/bin/bash",
        "# Distributor for federated " + fileConfig.name + ".lf Lingua Franca program.",
        "# Uncomment to specify to behave as close as possible to the POSIX standard.",
        "# set -o posix");
  }

  private String getRtiCommand(
      String rtiBinPath, List<FederateInstance> federates, boolean isRemote) {
    List<String> commands = new ArrayList<>();
    if (isRemote) {
      commands.add(rtiBinPath + " -i '${FEDERATION_ID}' \\");
    } else {
      commands.add(rtiBinPath + " -i ${FEDERATION_ID} \\");
    }
    if (targetConfig.getOrDefault(AuthProperty.INSTANCE)) {
      commands.add("                        -a \\");
    }
    if (targetConfig.getOrDefault(TracingProperty.INSTANCE).isEnabled()) {
      commands.add("                        -t \\");
    }
    if (!targetConfig.getOrDefault(DNETProperty.INSTANCE)) {
      commands.add("                        -d \\");
    }
    commands.addAll(
        List.of(
            "                        -n " + federates.size() + " \\",
            "                        -c "
                + targetConfig.getOrDefault(ClockSyncModeProperty.INSTANCE).toString()
                + " \\"));
    if (targetConfig.getOrDefault(ClockSyncModeProperty.INSTANCE).equals(ClockSyncMode.ON)) {
      commands.add(
          "period "
              + targetConfig.getOrDefault(ClockSyncOptionsProperty.INSTANCE).period.toNanoSeconds()
              + " \\");
    }
    if (targetConfig.getOrDefault(ClockSyncModeProperty.INSTANCE).equals(ClockSyncMode.ON)
        || targetConfig.getOrDefault(ClockSyncModeProperty.INSTANCE).equals(ClockSyncMode.INIT)) {
      commands.add(
          "exchanges-per-interval "
              + targetConfig.getOrDefault(ClockSyncOptionsProperty.INSTANCE).trials
              + " \\");
    }
    return String.join("\n", commands);
  }

  private String getLaunchCode(String rtiLaunchCode) {
    String launchCodeWithLogging = String.join(" ", rtiLaunchCode, ">& RTI.log &");
    String launchCodeWithoutLogging = String.join(" ", rtiLaunchCode, "&");
    return String.join(
        "\n",
        "echo \"#### Launching the runtime infrastructure (RTI).\"",
        "# The RTI is started first to allow proper boot-up",
        "# before federates will try to connect.",
        "# The RTI will be brought back to foreground",
        "# to be responsive to user inputs after all federates",
        "# are launched.",
        "if [ \"$1\" = \"-l\" ]; then",
        launchCodeWithLogging,
        "else",
        launchCodeWithoutLogging,
        "fi",
        "# Store the PID of the RTI",
        "RTI=$!",
        "# Wait for the RTI to boot up before",
        "# starting federates (this could be done by waiting for a specific output",
        "# from the RTI, but here we use sleep)",
        "sleep 1");
  }

  private String getRemoteLaunchCode(
      Object host, Object target, String logFileName, String rtiLaunchString) {
    return String.join(
        "\n",
        "echo \"#### Launching the runtime infrastructure (RTI) on remote host " + host + ".\"",
        "# FIXME: Killing this ssh does not kill the remote process.",
        "# A double -t -t option to ssh forces creation of a virtual terminal, which",
        "# fixes the problem, but then the ssh command does not execute. The remote",
        "# federate does not start!",
        "ssh " + target + " 'mkdir -p log; \\",
        "    echo \"-------------- Federation ID: \"'$FEDERATION_ID' >> " + logFileName + "; \\",
        "    date >> " + logFileName + "; \\",
        "    echo \"Executing RTI: "
            + rtiLaunchString
            + "\n\" 2>&1 | tee -a "
            + logFileName
            + "; \\",
        "    # First, check if the RTI is on the PATH",
        "    if ! command -v RTI &> /dev/null",
        "    then",
        "        echo \"RTI could not be found.\"",
        "        echo \"The source code can be found in org.lflang/src/lib/core/federated/RTI\"",
        "        exit 1",
        "    fi",
        "    " + rtiLaunchString + " 2>&1 | tee -a " + logFileName + "' &",
        "# Store the PID of the channel to RTI",
        "RTI=$!",
        "# Wait for the RTI to boot up before",
        "# starting federates (this could be done by waiting for a specific output",
        "# from the RTI, but here we use sleep)",
        "sleep 5");
  }

  /**
   * Return the shell commands that copy the generated source files for the specified federate onto
   * the remote host and compiles them on that host. When the shell script is executed on the local
   * machine, the source files will be put in `remoteBase/programName/federateName`. Also, a
   * `build.sh` script will be put in `remoteBase/programName/bin`. This script will be executed,
   * and its output will be put into a log file in `remoteBase/programName/log`.
   *
   * @param remoteBase The root directory on the remote machine.
   * @param federate The federate to distribute.
   */
  private String getDistCode(Path remoteBase, String name, String user, String host) {
    String binDirectory = "~/" + remoteBase + "/" + fileConfig.name + "/bin";
    String logDirectory = "~/" + remoteBase + "/" + fileConfig.name + "/log";
    String remoteBuildLogFileName = logDirectory + "/build.log";
    String buildShellFileName = "build_" + name + ".sh";
    String tarFileName = name + ".tar.gz";
    return String.join(
        "\n",
        "echo \"Making directory "
            + remoteBase
            + " and subdirectories federate_name, bin, and log on host "
            + getUserHost(user, host)
            + "\"",
        "ssh " + getUserHost(user, host) + " '\\",
        "    mkdir -p " + binDirectory + " " + logDirectory + "; \\",
        "    echo \"------Build of "
            + fileConfig.name
            + " "
            + name
            + "\" >> "
            + remoteBuildLogFileName
            + "; \\",
        "    date >> " + remoteBuildLogFileName + ";",
        "'",
        "pushd " + fileConfig.getSrcGenPath() + " > /dev/null",
        "echo \"**** Bundling source files into " + tarFileName + "\"",
        "tar -czf " + tarFileName + " --exclude build " + name,
        "echo \"**** Copying tarfile to host " + getUserHost(user, host) + "\"",
        "scp -r "
            + tarFileName
            + " "
            + getUserHost(user, host)
            + ":"
            + remoteBase
            + "/"
            + fileConfig.name
            + "/"
            + tarFileName,
        "rm " + tarFileName,
        "ssh " + getUserHost(user, host) + " '\\",
        "    cd ~/" + remoteBase + "/" + fileConfig.name + "; \\",
        "    tar -xzf " + tarFileName + "; \\",
        "    rm " + tarFileName + ";",
        "'",
        "popd > /dev/null",
        "echo \"**** Generating and executing compile.sh on host " + getUserHost(user, host) + "\"",
        "ssh "
            + getUserHost(user, host)
            + " '"
            + "cd "
            + remoteBase
            + "/"
            + fileConfig.name
            + "/bin; "
            + "rm -rf "
            + buildShellFileName
            + "; "
            // -l option ensures that the script runs as a login shell, getting PATh, etc.
            + "echo \"#!/bin/bash -l\" >> "
            + buildShellFileName
            + "; "
            + "chmod +x "
            + buildShellFileName
            + "; "
            + "echo \"# Build commands for "
            + fileConfig.name
            + " "
            + name
            + "\" >> "
            + buildShellFileName
            + "; "
            + "echo \"cd ~/"
            + remoteBase
            + "/"
            + fileConfig.name
            + "/"
            + name
            + "\" >> "
            + buildShellFileName
            + "; "
            // The >> syntax appends stdout to a file. The 2>&1 appends stderr to the same file. tee
            // sens to stdout and file.
            + "echo \"rm -rf build && mkdir -p build && cd build && cmake .. && make 2>&1 | tee -a "
            + remoteBuildLogFileName
            + "\" >> "
            + buildShellFileName
            + "; "
            + "echo \"mv "
            + name
            + " "
            + binDirectory
            + "\" >>  "
            + buildShellFileName
            + "; "
            + binDirectory
            + "/"
            + buildShellFileName
            + "'");
  }

  private String getUserHost(Object user, Object host) {
    if (user == null) {
      return host.toString();
    }
    return user + "@" + host;
  }

  /**
   * Return shell script code that launches the federate and captures its output to a log file.
   *
   * @param remoteBase The root directory on the remote machine.
   * @param federate The federate to distribute.
   * @param federateIndex The index of the federate in the order of launch.
   */
  private String getFedRemoteLaunchCode(
      Path remoteBase, FederateInstance federate, int federateIndex) {
    String logDirectory = "~/" + remoteBase + "/" + fileConfig.name + "/log";
    String runLogFileName = logDirectory + "/" + federate.name + ".log";
    String binDirectory = "~/" + remoteBase + "/" + fileConfig.name + "/bin";
    String executeCommand = binDirectory + "/" + federate.name + " -i '$FEDERATION_ID'";

    return String.join(
        "\n",
        "echo \"#### Launching the federate "
            + federate.name
            + " on host "
            + getUserHost(federate.user, federate.host)
            + "\"",
        "# FIXME: Killing this ssh does not kill the remote process.",
        "# A double -t -t option to ssh forces creation of a virtual terminal, which",
        "# fixes the problem, but then the ssh command does not execute. The remote",
        "# federate does not start!",
        "ssh " + getUserHost(federate.user, federate.host) + " '",
        "    cd " + remoteBase + "; \\",
        "    echo \"Executing: " + executeCommand + "\" 2>&1 | tee -a " + runLogFileName + "; ",
        "    " + executeCommand + " 2>&1 | tee -a " + runLogFileName + "' &",
        "pids[" + federateIndex + "]=$!");
  }

  private String getFedLocalLaunchCode(
      FederateInstance federate, String executeCommand, int federateIndex) {
    return String.join(
        "\n",
        "echo \"#### Launching the federate " + federate.name + ".\"",
        "if [ \"$1\" = \"-l\" ]; then",
        "    " + executeCommand + " >& " + federate.name + ".log &",
        "else",
        "    " + executeCommand + " &",
        "fi",
        "pids[" + federateIndex + "]=$!");
  }

  /**
   * Create a build configuration of the appropriate target.
   *
   * @param federate The federate to which the build configuration applies.
   * @param fileConfig The file configuration of the federation to which the federate belongs.
   */
  private BuildConfig getBuildConfig(
      FederateInstance federate, FederationFileConfig fileConfig, MessageReporter messageReporter) {
    return switch (federate.targetConfig.target) {
      case C, CCPP -> new CBuildConfig(federate, fileConfig, messageReporter);
      case Python -> new PyBuildConfig(federate, fileConfig, messageReporter);
      case TS -> new TsBuildConfig(federate, fileConfig, messageReporter);
      case CPP, Rust, UC -> throw new UnsupportedOperationException();
    };
  }
}
