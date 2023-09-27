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
import java.util.HashSet;
import java.util.List;
import java.util.Properties;
import java.util.Set;

import org.lflang.generator.LFGeneratorContext.BuildParm;
import org.lflang.generator.rust.RustTargetConfig;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.TargetDecl;
import org.lflang.target.property.AuthProperty;
import org.lflang.target.property.ClockSyncModeProperty;
import org.lflang.target.property.CoordinationModeProperty;
import org.lflang.target.property.CoordinationOptionsProperty;
import org.lflang.target.property.DockerProperty;
import org.lflang.target.property.FastProperty;
import org.lflang.target.property.KeepaliveProperty;
import org.lflang.target.property.PlatformProperty;
import org.lflang.target.property.Ros2DependenciesProperty;
import org.lflang.target.property.SchedulerProperty;
import org.lflang.target.property.LoggingProperty;
import org.lflang.target.property.LoggingProperty.LogLevel;
import org.lflang.target.property.TracingProperty;
import org.lflang.target.property.BuildCommandsProperty;
import org.lflang.target.property.ClockSyncOptionsProperty;
import org.lflang.target.property.BuildTypeProperty;
import org.lflang.target.property.CompilerFlagsProperty;
import org.lflang.target.property.ExportDependencyGraphProperty;
import org.lflang.target.property.ExportToYamlProperty;
import org.lflang.target.property.FilesProperty;
import org.lflang.target.property.NoCompileProperty;
import org.lflang.target.property.NoRuntimeValidationProperty;
import org.lflang.target.property.PrintStatisticsProperty;
import org.lflang.target.property.ProtobufsProperty;
import org.lflang.target.property.Ros2Property;
import org.lflang.target.property.RuntimeVersionProperty;
import org.lflang.target.property.SingleFileProjectProperty;
import org.lflang.target.property.ThreadingProperty;
import org.lflang.target.property.TimeOutProperty;
import org.lflang.target.property.WorkersProperty;
import org.lflang.target.property.CmakeIncludeProperty;
import org.lflang.target.property.type.CompileDefinitionsConfig;
import org.lflang.target.property.type.CompilerConfig;
import org.lflang.target.property.type.ExternalRuntimePathConfig;
import org.lflang.target.property.type.VerifyProperty;

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

    // FIXME: work these into the TargetProperty.set call above.

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
  public BuildCommandsProperty buildCommands = new BuildCommandsProperty();

  /**
   * The mode of clock synchronization to be used in federated programs. The default is 'initial'.
   */
  public final ClockSyncModeProperty clockSync = new ClockSyncModeProperty();

  /** Clock sync options. */
  public final ClockSyncOptionsProperty clockSyncOptions = new ClockSyncOptionsProperty();

  /** Parameter passed to cmake. The default is 'Release'. */
  public BuildTypeProperty buildType = new BuildTypeProperty();

  /** Optional additional extensions to include in the generated CMakeLists.txt. */
  public CmakeIncludeProperty cmakeIncludes = new CmakeIncludeProperty();

  /** The compiler to invoke, unless a build command has been specified. */
  public CompilerConfig compiler = new CompilerConfig();

  /** Additional sources to add to the compile command if appropriate. */
  public List<String> compileAdditionalSources = new ArrayList<>();

  /**
   * Additional (preprocessor) definitions to add to the compile command if appropriate.
   *
   * <p>The first string is the definition itself, and the second string is the value to attribute
   * to that definition, if any. The second value could be left empty.
   */
  public CompileDefinitionsConfig compileDefinitions = new CompileDefinitionsConfig();

  /** Flags to pass to the compiler, unless a build command has been specified. */
  public CompilerFlagsProperty compilerFlags = new CompilerFlagsProperty();

  /**
   * The type of coordination used during the execution of a federated program. The default is
   * 'centralized'.
   */
  public CoordinationModeProperty coordination = new CoordinationModeProperty();

  /** Docker options. */
  public DockerProperty dockerOptions = new DockerProperty();

  /** Coordination options. */
  public CoordinationOptionsProperty coordinationOptions = new CoordinationOptionsProperty();

  /** Link to an external runtime library instead of the default one. */
  public ExternalRuntimePathConfig externalRuntimePath = new ExternalRuntimePathConfig();

  /**
   * If true, configure the execution environment such that it does not wait for physical time to
   * match logical time. The default is false.
   */
  public FastProperty fastMode = new FastProperty();

  /** List of files to be copied to src-gen. */
  public FilesProperty files = new FilesProperty();

  /**
   * If true, configure the execution environment to keep executing if there are no more events on
   * the event queue. The default is false.
   */
  public KeepaliveProperty keepalive = new KeepaliveProperty();

  /** The level of logging during execution. The default is INFO. */
  public LoggingProperty logLevel = new LoggingProperty();

  /** Flags to pass to the linker, unless a build command has been specified. */
  public String linkerFlags = "";

  /** If true, do not invoke the target compiler or build command. The default is false. */
  public NoCompileProperty noCompile = new NoCompileProperty();

  /** If true, do not perform runtime validation. The default is false. */
  public NoRuntimeValidationProperty noRuntimeValidation = new NoRuntimeValidationProperty();

  /** If true, check the generated verification model. The default is false. */
  public VerifyProperty verify = new VerifyProperty();

  /**
   * Set the target platform config. This tells the build system what platform-specific support
   * files it needs to incorporate at compile time.
   *
   * <p>This is now a wrapped class to account for overloaded definitions of defining platform
   * (either a string or dictionary of values)
   */
  public PlatformProperty platformOptions = new PlatformProperty();

  /** If true, instruct the runtime to collect and print execution statistics. */
  public PrintStatisticsProperty printStatistics = new PrintStatisticsProperty();

  /** List of proto files to be processed by the code generator. */
  public ProtobufsProperty protoFiles = new ProtobufsProperty();

  /** If true, generate ROS2 specific code. */
  public Ros2Property ros2 = new Ros2Property();

  /** Additional ROS2 packages that the LF program depends on. */
  public Ros2DependenciesProperty ros2Dependencies = new Ros2DependenciesProperty();

  /** The version of the runtime library to be used in the generated target. */
  public RuntimeVersionProperty runtimeVersion = new RuntimeVersionProperty();

  /** Whether all reactors are to be generated into a single target language file. */
  public SingleFileProjectProperty singleFileProject = new SingleFileProjectProperty();

  /** What runtime scheduler to use. */
  public SchedulerProperty schedulerType = new SchedulerProperty();

  /**
   * The number of worker threads to deploy. The default is zero, which indicates that the runtime
   * is allowed to freely choose the number of workers.
   */
  public WorkersProperty workers = new WorkersProperty();

  /** Indicate whether HMAC authentication is used. */
  public AuthProperty auth = new AuthProperty();

  /** Indicate whether the runtime should use multithreaded execution. */
  public ThreadingProperty threading = new ThreadingProperty();

  /** The timeout to be observed during execution of the program. */
  public TimeOutProperty timeout = new TimeOutProperty();

  /** If non-null, configure the runtime environment to perform tracing. The default is null. */
  public TracingProperty tracing = new TracingProperty();

  /**
   * If true, the resulting binary will output a graph visualizing all reaction dependencies.
   *
   * <p>This option is currently only used for C++ and Rust. This export function is a valuable tool
   * for debugging LF programs and helps to understand the dependencies inferred by the runtime.
   */
  public ExportDependencyGraphProperty exportDependencyGraph = new ExportDependencyGraphProperty();

  /**
   * If true, the resulting binary will output a yaml file describing the whole reactor structure of
   * the program.
   *
   * <p>This option is currently only used for C++. This export function is a valuable tool for
   * debugging LF programs and performing external analysis.
   */
  public ExportToYamlProperty exportToYaml = new ExportToYamlProperty();

  /** Rust-specific configuration. */
  public final RustTargetConfig rust =
      new RustTargetConfig(); // FIXME: https://issue.lf-lang.org/1558

  /** Path to a C file used by the Python target to setup federated execution. */
  public String fedSetupPreamble = null; // FIXME: https://issue.lf-lang.org/1558

}
