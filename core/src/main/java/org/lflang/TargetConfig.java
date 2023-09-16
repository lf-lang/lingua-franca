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
import java.util.Properties;
import java.util.Set;
import org.lflang.generator.LFGeneratorContext.BuildParm;
import org.lflang.generator.rust.RustTargetConfig;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.TargetDecl;
import org.lflang.target.AuthConfig;
import org.lflang.target.ClockSyncModeConfig;
import org.lflang.target.CoordinationModeConfig;
import org.lflang.target.CoordinationOptionsConfig;
import org.lflang.target.DockerConfig;
import org.lflang.target.FastModeConfig;
import org.lflang.target.KeepaliveConfig;
import org.lflang.target.PlatformConfig;
import org.lflang.target.SchedulerConfig;
import org.lflang.target.TracingConfig;
import org.lflang.target.TracingConfig.TracingOptions;
import org.lflang.target.property.BuildCommandsConfig;
import org.lflang.target.LoggingConfigurator.LogLevel;
import org.lflang.target.SchedulerConfig.SchedulerOption;
import org.lflang.target.property.ClockSyncOptionsConfig;
import org.lflang.target.property.BuildTypeConfig;

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

    if (cliArgs != null) {
      TargetProperty.override(this, cliArgs, messageReporter);
    }

    if (cliArgs.containsKey("no-compile")) {
      this.noCompile = true;
    }
    if (cliArgs.containsKey("verify")) {
      this.verify = true;
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
      this.tracing.override(new TracingOptions());
      this.setByUser.add(TargetProperty.TRACING);
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
  public BuildCommandsConfig buildCommands = new BuildCommandsConfig();

  /**
   * The mode of clock synchronization to be used in federated programs. The default is 'initial'.
   */
  public final ClockSyncModeConfig clockSync = new ClockSyncModeConfig();

  /** Clock sync options. */
  public final ClockSyncOptionsConfig clockSyncOptions = new ClockSyncOptionsConfig();

  /** Parameter passed to cmake. The default is 'Release'. */
  public BuildTypeConfig buildType = new BuildTypeConfig();

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
  public CoordinationModeConfig coordination = new CoordinationModeConfig();

  /** Docker options. */
  public DockerConfig dockerOptions = new DockerConfig();

  /** Coordination options. */
  public CoordinationOptionsConfig coordinationOptions = new CoordinationOptionsConfig();

  /** Link to an external runtime library instead of the default one. */
  public String externalRuntimePath = null;

  /**
   * If true, configure the execution environment such that it does not wait for physical time to
   * match logical time. The default is false.
   */
  public FastModeConfig fastMode = new FastModeConfig();

  /** List of files to be copied to src-gen. */
  public List<String> files = new ArrayList<>();

  /**
   * If true, configure the execution environment to keep executing if there are no more events on
   * the event queue. The default is false.
   */
  public KeepaliveConfig keepalive = new KeepaliveConfig();

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
  public PlatformConfig platformOptions = new PlatformConfig();

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
  public SchedulerConfig schedulerType = new SchedulerConfig();

  /**
   * The number of worker threads to deploy. The default is zero, which indicates that the runtime
   * is allowed to freely choose the number of workers.
   */
  public int workers = 0;

  /** Indicate whether HMAC authentication is used. */
  public AuthConfig auth = new AuthConfig();

  /** Indicate whether the runtime should use multithreaded execution. */
  public boolean threading = true;

  /** The timeout to be observed during execution of the program. */
  public TimeValue timeout;

  /** If non-null, configure the runtime environment to perform tracing. The default is null. */
  public TracingConfig tracing = new TracingConfig();

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






}
