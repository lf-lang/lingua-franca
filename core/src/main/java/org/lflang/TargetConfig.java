/*************
 * Copyright (c) 2019, The University of California at Berkeley.
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
package org.lflang;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Properties;
import java.util.Set;
import org.lflang.TargetProperty.BuildType;
import org.lflang.TargetProperty.ClockSyncMode;
import org.lflang.TargetProperty.CoordinationType;
import org.lflang.TargetProperty.LogLevel;
import org.lflang.TargetProperty.Platform;
import org.lflang.TargetProperty.SchedulerOption;
import org.lflang.TargetProperty.UnionType;
import org.lflang.generator.LFGeneratorContext.BuildParm;
import org.lflang.generator.rust.RustTargetConfig;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.TargetDecl;

/**
 * A class for keeping the current target configuration.
 *
 * <p>Class members of type String are initialized as empty strings, unless otherwise stated.
 *
 * @author Marten Lohstroh
 */
public class TargetConfig {

  /** The target of this configuration (e.g., C, TypeScript, Python). */
  public final Target target;

  /**
   * Create a new target configuration based on the given target declaration AST node only.
   *
   * @param target AST node of a target declaration.
   */
  public TargetConfig(TargetDecl target) { // FIXME: eliminate this constructor if we can
    this.target = Target.fromDecl(target);
  }

  /**
   * Create a new target configuration based on the given commandline arguments and target
   * declaration AST node.
   *
   * @param cliArgs Arguments passed on the commandline.
   * @param target AST node of a target declaration.
   * @param messageReporter An error reporter to report problems.
   */
  public TargetConfig(Properties cliArgs, TargetDecl target, MessageReporter messageReporter) {
    this(target);
    if (target.getConfig() != null) {
      List<KeyValuePair> pairs = target.getConfig().getPairs();
      TargetProperty.set(this, pairs != null ? pairs : List.of(), messageReporter);
    }
    if (cliArgs.containsKey("no-compile")) {
      this.noCompile = true;
    }
    if (cliArgs.containsKey("verify")) {
      this.verify = true;
    }
    if (cliArgs.containsKey("docker")) {
      var arg = cliArgs.getProperty("docker");
      if (Boolean.parseBoolean(arg)) {
        this.dockerOptions = new DockerOptions();
      } else {
        this.dockerOptions = null;
      }
      // FIXME: this is pretty ad-hoc and does not account for more complex overrides yet.
    }
    if (cliArgs.containsKey("build-type")) {
      this.cmakeBuildType =
          (BuildType) UnionType.BUILD_TYPE_UNION.forName(cliArgs.getProperty("build-type"));
      this.setByUser.add(TargetProperty.BUILD_TYPE);
    }
    if (cliArgs.containsKey("logging")) {
      this.logLevel = LogLevel.valueOf(cliArgs.getProperty("logging").toUpperCase());
      this.setByUser.add(TargetProperty.LOGGING);
    }
    if (cliArgs.containsKey("workers")) {
      this.workers = Integer.parseInt(cliArgs.getProperty("workers"));
      this.setByUser.add(TargetProperty.WORKERS);
    }
    if (cliArgs.containsKey("threading")) {
      this.threading = Boolean.parseBoolean(cliArgs.getProperty("threading"));
      this.setByUser.add(TargetProperty.THREADING);
    }
    if (cliArgs.containsKey("target-compiler")) {
      this.compiler = cliArgs.getProperty("target-compiler");
      this.setByUser.add(TargetProperty.COMPILER);
    }
    if (cliArgs.containsKey("tracing")) {
      this.tracing = new TracingOptions();
      this.setByUser.add(TargetProperty.TRACING);
    }
    if (cliArgs.containsKey("scheduler")) {
      this.schedulerType = SchedulerOption.valueOf(cliArgs.getProperty("scheduler"));
      this.setByUser.add(TargetProperty.SCHEDULER);
    }
    if (cliArgs.containsKey("target-flags")) {
      this.compilerFlags.clear();
      if (!cliArgs.getProperty("target-flags").isEmpty()) {
        this.compilerFlags.addAll(List.of(cliArgs.getProperty("target-flags").split(" ")));
      }
      this.setByUser.add(TargetProperty.FLAGS);
    }
    if (cliArgs.containsKey("runtime-version")) {
      this.runtimeVersion = cliArgs.getProperty("runtime-version");
      this.setByUser.add(TargetProperty.RUNTIME_VERSION);
    }
    if (cliArgs.containsKey("external-runtime-path")) {
      this.externalRuntimePath = cliArgs.getProperty("external-runtime-path");
      this.setByUser.add(TargetProperty.EXTERNAL_RUNTIME_PATH);
    }
    if (cliArgs.containsKey(TargetProperty.KEEPALIVE.description)) {
      this.keepalive =
          Boolean.parseBoolean(cliArgs.getProperty(TargetProperty.KEEPALIVE.description));
      this.setByUser.add(TargetProperty.KEEPALIVE);
    }
    if (cliArgs.containsKey(BuildParm.PRINT_STATISTICS.getKey())) {
      this.printStatistics = true;
      this.setByUser.add(TargetProperty.PRINT_STATISTICS);
    }
  }

  /** Keep track of every target property that is explicitly set by the user. */
  public Set<TargetProperty> setByUser = new HashSet<>();

  /**
   * A list of custom build commands that replace the default build process of directly invoking a
   * designated compiler. A common usage of this target property is to set the command to build on
   * the basis of a Makefile.
   */
  public List<String> buildCommands = new ArrayList<>();

  /**
   * The mode of clock synchronization to be used in federated programs. The default is 'initial'.
   */
  public ClockSyncMode clockSync = ClockSyncMode.INIT;

  /** Clock sync options. */
  public ClockSyncOptions clockSyncOptions = new ClockSyncOptions();

  /** Parameter passed to cmake. The default is 'Release'. */
  public BuildType cmakeBuildType = BuildType.RELEASE;

  /** Optional additional extensions to include in the generated CMakeLists.txt. */
  public List<String> cmakeIncludes = new ArrayList<>();

  /** The compiler to invoke, unless a build command has been specified. */
  public String compiler = "";

  /** Additional sources to add to the compile command if appropriate. */
  public List<String> compileAdditionalSources = new ArrayList<>();

  /**
   * Additional (preprocessor) definitions to add to the compile command if appropriate.
   *
   * <p>The first string is the definition itself, and the second string is the value to attribute
   * to that definition, if any. The second value could be left empty.
   */
  public Map<String, String> compileDefinitions = new HashMap<>();

  /** Flags to pass to the compiler, unless a build command has been specified. */
  public List<String> compilerFlags = new ArrayList<>();

  /**
   * The type of coordination used during the execution of a federated program. The default is
   * 'centralized'.
   */
  public CoordinationType coordination = CoordinationType.CENTRALIZED;

  /** Docker options. */
  public DockerOptions dockerOptions = null;

  /** Coordination options. */
  public CoordinationOptions coordinationOptions = new CoordinationOptions();

  /** Link to an external runtime library instead of the default one. */
  public String externalRuntimePath = null;

  /**
   * If true, configure the execution environment such that it does not wait for physical time to
   * match logical time. The default is false.
   */
  public boolean fastMode = false;

  /** List of files to be copied to src-gen. */
  public List<String> files = new ArrayList<>();

  /**
   * If true, configure the execution environment to keep executing if there are no more events on
   * the event queue. The default is false.
   */
  public boolean keepalive = false;

  /** The level of logging during execution. The default is INFO. */
  public LogLevel logLevel = LogLevel.INFO;

  /** Flags to pass to the linker, unless a build command has been specified. */
  public String linkerFlags = "";

  /** If true, do not invoke the target compiler or build command. The default is false. */
  public boolean noCompile = false;

  /** If true, do not perform runtime validation. The default is false. */
  public boolean noRuntimeValidation = false;

  /** If true, check the generated verification model. The default is false. */
  public boolean verify = false;

  /**
   * Set the target platform config. This tells the build system what platform-specific support
   * files it needs to incorporate at compile time.
   *
   * <p>This is now a wrapped class to account for overloaded definitions of defining platform
   * (either a string or dictionary of values)
   */
  public PlatformOptions platformOptions = new PlatformOptions();

  /** If true, instruct the runtime to collect and print execution statistics. */
  public boolean printStatistics = false;

  /** List of proto files to be processed by the code generator. */
  public List<String> protoFiles = new ArrayList<>();

  /** If true, generate ROS2 specific code. */
  public boolean ros2 = false;

  /** Additional ROS2 packages that the LF program depends on. */
  public List<String> ros2Dependencies = null;

  /** The version of the runtime library to be used in the generated target. */
  public String runtimeVersion = null;

  /** Whether all reactors are to be generated into a single target language file. */
  public boolean singleFileProject = false;

  /** What runtime scheduler to use. */
  public SchedulerOption schedulerType = SchedulerOption.getDefault();

  /**
   * The number of worker threads to deploy. The default is zero, which indicates that the runtime
   * is allowed to freely choose the number of workers.
   */
  public int workers = 0;

  /** Indicate whether HMAC authentication is used. */
  public boolean auth = false;

  /** Indicate whether the runtime should use multithreaded execution. */
  public boolean threading = true;

  /** The timeout to be observed during execution of the program. */
  public TimeValue timeout;

  /** If non-null, configure the runtime environment to perform tracing. The default is null. */
  public TracingOptions tracing = null;

  /**
   * If true, the resulting binary will output a graph visualizing all reaction dependencies.
   *
   * <p>This option is currently only used for C++ and Rust. This export function is a valuable tool
   * for debugging LF programs and helps to understand the dependencies inferred by the runtime.
   */
  public boolean exportDependencyGraph = false;

  /**
   * If true, the resulting binary will output a yaml file describing the whole reactor structure of
   * the program.
   *
   * <p>This option is currently only used for C++. This export function is a valuable tool for
   * debugging LF programs and performing external analysis.
   */
  public boolean exportToYaml = false;

  /** Rust-specific configuration. */
  public final RustTargetConfig rust =
      new RustTargetConfig(); // FIXME: https://issue.lf-lang.org/1558

  /** Path to a C file used by the Python target to setup federated execution. */
  public String fedSetupPreamble = null; // FIXME: https://issue.lf-lang.org/1558

  /** Settings related to clock synchronization. */
  public static class ClockSyncOptions {

    /**
     * Dampen the adjustments to the clock synchronization offset by this rate. The default is 10.
     */
    public int attenuation = 10;

    /**
     * Whether to collect statistics while performing clock synchronization. This setting is only
     * considered when clock synchronization has been activated. The default is true.
     */
    public boolean collectStats = true;

    /** Enable clock synchronization for federates on the same machine. Default is false. */
    public boolean localFederatesOn = false;

    /**
     * Interval at which clock synchronization is initiated by the RTI (will be passed to it as an
     * argument on the command-line). The default is 5 milliseconds.
     */
    public TimeValue period = new TimeValue(5, TimeUnit.MILLI);

    /**
     * Indicate the number of exchanges to be had per each clock synchronization round. See
     * /lib/core/federated/clock-sync.h for more details. The default is 10.
     */
    public int trials = 10;

    /**
     * Used to create an artificial clock synchronization error for the purpose of testing. The
     * default is null.
     */
    public TimeValue testOffset;
  }

  /** Settings related to coordination of federated execution. */
  public static class CoordinationOptions {

    /**
     * For centralized coordination, if a federate has a physical action that can trigger an output,
     * directly or indirectly, then it will send NET (next event tag) messages to the RTI
     * periodically as its physical clock advances. This option sets the amount of time to wait
     * between sending such messages. Increasing this value results in downstream federates that lag
     * further behind physical time (if the "after" delays are insufficient). The default is null,
     * which means it is up the implementation to choose an interval.
     */
    public TimeValue advance_message_interval = null;
  }

  /** Settings related to Docker options. */
  public static class DockerOptions {
    /**
     * The base image and tag from which to build the Docker image. The default is "alpine:latest".
     */
    public String from = "alpine:latest";

    @Override
    public boolean equals(Object o) {
      if (this == o) {
        return true;
      }
      if (o == null || getClass() != o.getClass()) {
        return false;
      }
      DockerOptions that = (DockerOptions) o;
      return from.equals(that.from);
    }
  }

  /** Settings related to Platform Options. */
  public static class PlatformOptions {

    /**
     * The base platform we build our LF Files on. Should be set to AUTO by default unless
     * developing for specific OS/Embedded Platform
     */
    public Platform platform = Platform.AUTO;

    /**
     * The string value used to determine what type of embedded board we work with and can be used
     * to simplify the build process. For example, when we want to flash to an Arduino Nano 33 BLE
     * board, we can use the string arduino:mbed_nano:nano33ble
     */
    public String board = null;

    /**
     * The string value used to determine the port on which to flash the compiled program (i.e.
     * /dev/cu.usbmodem21301)
     */
    public String port = null;

    /**
     * The baud rate used as a parameter to certain embedded platforms. 9600 is a standard rate
     * amongst systems like Arduino, so it's the default value.
     */
    public int baudRate = 9600;

    /**
     * The boolean statement used to determine whether we should automatically attempt to flash once
     * we compile. This may require the use of board and port values depending on the infrastructure
     * you use to flash the boards.
     */
    public boolean flash = false;

    /**
     * The int value is used to determine the number of needed threads for the user application in
     * Zephyr.
     */
    public int userThreads = 0;
  }

  /** Settings related to tracing options. */
  public static class TracingOptions {
    /**
     * The name to use as the root of the trace file produced. This defaults to the name of the .lf
     * file.
     */
    public String traceFileName = null;

    @Override
    public boolean equals(Object o) {
      if (this == o) {
        return true;
      }
      if (o == null || getClass() != o.getClass()) {
        return false;
      }
      TracingOptions that = (TracingOptions) o;
      return Objects.equals(traceFileName, that.traceFileName); // traceFileName may be null
    }
  }
}
