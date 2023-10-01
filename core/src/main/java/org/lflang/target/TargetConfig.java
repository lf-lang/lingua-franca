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
package org.lflang.target;

import java.util.ArrayList;
import java.util.List;
import java.util.Properties;

import org.lflang.MessageReporter;
import org.lflang.Target;
import org.lflang.TargetProperty;
import org.lflang.generator.rust.RustTargetConfig;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.TargetDecl;
import org.lflang.target.property.AuthProperty;
import org.lflang.target.property.BuildCommandsProperty;
import org.lflang.target.property.BuildTypeProperty;
import org.lflang.target.property.ClockSyncModeProperty;
import org.lflang.target.property.ClockSyncOptionsProperty;
import org.lflang.target.property.CmakeIncludeProperty;
import org.lflang.target.property.CompileDefinitionsProperty;
import org.lflang.target.property.CompilerFlagsProperty;
import org.lflang.target.property.CompilerProperty;
import org.lflang.target.property.CoordinationOptionsProperty;
import org.lflang.target.property.CoordinationProperty;
import org.lflang.target.property.DockerProperty;
import org.lflang.target.property.ExportDependencyGraphProperty;
import org.lflang.target.property.ExportToYamlProperty;
import org.lflang.target.property.ExternalRuntimePathProperty;
import org.lflang.target.property.FastProperty;
import org.lflang.target.property.FedSetupProperty;
import org.lflang.target.property.FilesProperty;
import org.lflang.target.property.KeepaliveProperty;
import org.lflang.target.property.LoggingProperty;
import org.lflang.target.property.NoCompileProperty;
import org.lflang.target.property.NoRuntimeValidationProperty;
import org.lflang.target.property.PlatformProperty;
import org.lflang.target.property.PrintStatisticsProperty;
import org.lflang.target.property.ProtobufsProperty;
import org.lflang.target.property.Ros2DependenciesProperty;
import org.lflang.target.property.Ros2Property;
import org.lflang.target.property.RuntimeVersionProperty;
import org.lflang.target.property.SchedulerProperty;
import org.lflang.target.property.SingleFileProjectProperty;
import org.lflang.target.property.ThreadingProperty;
import org.lflang.target.property.TimeOutProperty;
import org.lflang.target.property.TracingProperty;
import org.lflang.target.property.WorkersProperty;
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
      TargetProperty.load(this, pairs, messageReporter);
    }

    if (cliArgs != null) {
      TargetProperty.load(this, cliArgs, messageReporter);
    }
  }

  /**
   * A list of custom build commands that replace the default build process of directly invoking a
   * designated compiler. A common usage of this target property is to set the command to build on
   * the basis of a Makefile.
   */
  public final BuildCommandsProperty buildCommands = new BuildCommandsProperty();

  /**
   * The mode of clock synchronization to be used in federated programs. The default is 'initial'.
   */
  public final ClockSyncModeProperty clockSync = new ClockSyncModeProperty();

  /** Clock sync options. */
  public final ClockSyncOptionsProperty clockSyncOptions = new ClockSyncOptionsProperty();

  /** Parameter passed to cmake. The default is 'Release'. */
  public final BuildTypeProperty buildType = new BuildTypeProperty();

  /** Optional additional extensions to include in the generated CMakeLists.txt. */
  public final CmakeIncludeProperty cmakeIncludes = new CmakeIncludeProperty();

  /** The compiler to invoke, unless a build command has been specified. */
  public final CompilerProperty compiler = new CompilerProperty();

  /** Additional sources to add to the compile command if appropriate. */
  public final List<String> compileAdditionalSources = new ArrayList<>();

  /**
   * Additional (preprocessor) definitions to add to the compile command if appropriate.
   *
   * <p>The first string is the definition itself, and the second string is the value to attribute
   * to that definition, if any. The second value could be left empty.
   */
  public final CompileDefinitionsProperty compileDefinitions = new CompileDefinitionsProperty();

  /** Flags to pass to the compiler, unless a build command has been specified. */
  public final CompilerFlagsProperty compilerFlags = new CompilerFlagsProperty();

  /**
   * The type of coordination used during the execution of a federated program. The default is
   * 'centralized'.
   */
  public final CoordinationProperty coordination = new CoordinationProperty();

  /** Docker options. */
  public final DockerProperty dockerOptions = new DockerProperty();

  /** Coordination options. */
  public final CoordinationOptionsProperty coordinationOptions = new CoordinationOptionsProperty();

  /** Link to an external runtime library instead of the default one. */
  public final ExternalRuntimePathProperty externalRuntimePath = new ExternalRuntimePathProperty();

  /**
   * If true, configure the execution environment such that it does not wait for physical time to
   * match logical time. The default is false.
   */
  public final FastProperty fastMode = new FastProperty();

  /** List of files to be copied to src-gen. */
  public final FilesProperty files = new FilesProperty();

  /**
   * If true, configure the execution environment to keep executing if there are no more events on
   * the event queue. The default is false.
   */
  public final KeepaliveProperty keepalive = new KeepaliveProperty();

  /** The level of logging during execution. The default is INFO. */
  public final LoggingProperty logLevel = new LoggingProperty();

  /** Flags to pass to the linker, unless a build command has been specified. */
  public String linkerFlags = "";

  /** If true, do not invoke the target compiler or build command. The default is false. */
  public final NoCompileProperty noCompile = new NoCompileProperty();

  /** If true, do not perform runtime validation. The default is false. */
  public final NoRuntimeValidationProperty noRuntimeValidation = new NoRuntimeValidationProperty();

  /** If true, check the generated verification model. The default is false. */
  public final VerifyProperty verify = new VerifyProperty();

  /**
   * Set the target platform config. This tells the build system what platform-specific support
   * files it needs to incorporate at compile time.
   *
   * <p>This is now a wrapped class to account for overloaded definitions of defining platform
   * (either a string or dictionary of values)
   */
  public final PlatformProperty platformOptions = new PlatformProperty();

  /** If true, instruct the runtime to collect and print execution statistics. */
  public final PrintStatisticsProperty printStatistics = new PrintStatisticsProperty();

  /** List of proto files to be processed by the code generator. */
  public final ProtobufsProperty protoFiles = new ProtobufsProperty();

  /** If true, generate ROS2 specific code. */
  public final Ros2Property ros2 = new Ros2Property();

  /** Additional ROS2 packages that the LF program depends on. */
  public final Ros2DependenciesProperty ros2Dependencies = new Ros2DependenciesProperty();

  /** The version of the runtime library to be used in the generated target. */
  public final RuntimeVersionProperty runtimeVersion = new RuntimeVersionProperty();

  /** Whether all reactors are to be generated into a single target language file. */
  public final SingleFileProjectProperty singleFileProject = new SingleFileProjectProperty();

  /** What runtime scheduler to use. */
  public final SchedulerProperty schedulerType = new SchedulerProperty();

  /**
   * The number of worker threads to deploy. The default is zero, which indicates that the runtime
   * is allowed to freely choose the number of workers.
   */
  public final WorkersProperty workers = new WorkersProperty();

  /** Indicate whether HMAC authentication is used. */
  public final AuthProperty auth = new AuthProperty();

  /** Indicate whether the runtime should use multithreaded execution. */
  public final ThreadingProperty threading = new ThreadingProperty();

  /** The timeout to be observed during execution of the program. */
  public final TimeOutProperty timeout = new TimeOutProperty();

  /** If non-null, configure the runtime environment to perform tracing. The default is null. */
  public final TracingProperty tracing = new TracingProperty();

  /**
   * If true, the resulting binary will output a graph visualizing all reaction dependencies.
   *
   * <p>This option is currently only used for C++ and Rust. This export function is a valuable tool
   * for debugging LF programs and helps to understand the dependencies inferred by the runtime.
   */
  public final ExportDependencyGraphProperty exportDependencyGraph =
      new ExportDependencyGraphProperty();

  /**
   * If true, the resulting binary will output a yaml file describing the whole reactor structure of
   * the program.
   *
   * <p>This option is currently only used for C++. This export function is a valuable tool for
   * debugging LF programs and performing external analysis.
   */
  public final ExportToYamlProperty exportToYaml = new ExportToYamlProperty();

  /** Rust-specific configuration. */
  public final RustTargetConfig rust = new RustTargetConfig();

  /** Path to a C file used by the Python target to setup federated execution. */
  public final FedSetupProperty fedSetupPreamble = new FedSetupProperty();
}
