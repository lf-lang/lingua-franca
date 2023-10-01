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

import java.nio.file.Path;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;
import java.util.Objects;
import java.util.Properties;
import java.util.stream.Collectors;
import org.lflang.ast.ASTUtils;
import org.lflang.generator.InvalidLfSourceException;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.lf.LfPackage.Literals;
import org.lflang.lf.Model;
import org.lflang.lf.TargetDecl;
import org.lflang.target.TargetConfig;
import org.lflang.target.property.AuthProperty;
import org.lflang.target.property.BuildCommandsProperty;
import org.lflang.target.property.BuildTypeProperty;
import org.lflang.target.property.CargoDependenciesProperty;
import org.lflang.target.property.CargoFeaturesProperty;
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
import org.lflang.target.property.RustIncludeProperty;
import org.lflang.target.property.SchedulerProperty;
import org.lflang.target.property.SingleFileProjectProperty;
import org.lflang.target.property.ThreadingProperty;
import org.lflang.target.property.TimeOutProperty;
import org.lflang.target.property.TracingProperty;
import org.lflang.target.property.WorkersProperty;
import org.lflang.target.property.type.VerifyProperty;
import org.lflang.validation.ValidatorMessageReporter;

/**
 * A target properties along with a type and a list of supporting targets that supports it, as well
 * as a function for configuration updates.
 *
 * @author Marten Lohstroh
 */
public enum TargetProperty {

  /** Directive to allow including OpenSSL libraries and process HMAC authentication. */
  AUTH(AuthProperty.class, config -> config.auth),
  /** Directive to let the generator use the custom build command. */
  BUILD(BuildCommandsProperty.class, config -> config.buildCommands),

  /**
   * Directive to specify the target build type such as 'Release' or 'Debug'. This is also used in
   * the Rust target to select a Cargo profile.
   */
  BUILD_TYPE(BuildTypeProperty.class, config -> config.buildType),

  /** Directive to let the federate execution handle clock synchronization in software. */
  CLOCK_SYNC(ClockSyncModeProperty.class, config -> config.clockSync),
  /** Key-value pairs giving options for clock synchronization. */
  CLOCK_SYNC_OPTIONS(ClockSyncOptionsProperty.class, config -> config.clockSyncOptions),

  /**
   * Directive to specify a cmake to be included by the generated build systems.
   *
   * <p>This gives full control over the C/C++ build as any cmake parameters can be adjusted in the
   * included file.
   */
  CMAKE_INCLUDE(CmakeIncludeProperty.class, config -> config.cmakeIncludes),
  /** Directive to specify the target compiler. */
  COMPILER(CompilerProperty.class, config -> config.compiler),
  /** Directive to specify compile-time definitions. */
  COMPILE_DEFINITIONS(CompileDefinitionsProperty.class, config -> config.compileDefinitions),

  /** Directive to specify the coordination mode */
  COORDINATION(CoordinationProperty.class, config -> config.coordination),
  /** Key-value pairs giving options for clock synchronization. */
  COORDINATION_OPTIONS(CoordinationOptionsProperty.class, config -> config.coordinationOptions),
  /**
   * Directive to generate a Dockerfile. This is either a boolean, true or false, or a dictionary of
   * options.
   */
  DOCKER(DockerProperty.class, config -> config.dockerOptions),
  /** Directive for specifying a path to an external runtime to be used for the compiled binary. */
  EXTERNAL_RUNTIME_PATH(ExternalRuntimePathProperty.class, config -> config.externalRuntimePath),
  /**
   * Directive to let the execution engine allow logical time to elapse faster than physical time.
   */
  FAST(FastProperty.class, config -> config.fastMode),
  /**
   * Directive to stage particular files on the class path to be processed by the code generator.
   */
  FILES(FilesProperty.class, config -> config.files),

  /** Flags to be passed on to the target compiler. */
  FLAGS(CompilerFlagsProperty.class, config -> config.compilerFlags),

  /**
   * Directive to let the execution engine remain active also if there are no more events in the
   * event queue.
   */
  KEEPALIVE(KeepaliveProperty.class, config -> config.keepalive),

  /** Directive to specify the grain at which to report log messages during execution. */
  LOGGING(LoggingProperty.class, config -> config.logLevel),

  /** Directive to not invoke the target compiler. */
  NO_COMPILE(NoCompileProperty.class, config -> config.noCompile),

  /** Directive to disable validation of reactor rules at runtime. */
  NO_RUNTIME_VALIDATION(NoRuntimeValidationProperty.class, config -> config.noRuntimeValidation),

  /**
   * Directive to specify the platform for cross code generation. This is either a string of the
   * platform or a dictionary of options that includes the string name.
   */
  PLATFORM(PlatformProperty.class, config -> config.platformOptions),

  /** Directive to instruct the runtime to collect and print execution statistics. */
  PRINT_STATISTICS(PrintStatisticsProperty.class, config -> config.printStatistics),

  /**
   * Directive for specifying .proto files that need to be compiled and their code included in the
   * sources.
   */
  PROTOBUFS(ProtobufsProperty.class, config -> config.protoFiles),

  /** Directive to specify that ROS2 specific code is generated, */
  ROS2(Ros2Property.class, config -> config.ros2),

  /** Directive to specify additional ROS2 packages that this LF program depends on. */
  ROS2_DEPENDENCIES(Ros2DependenciesProperty.class, config -> config.ros2Dependencies),

  /** Directive for specifying a specific version of the reactor runtime library. */
  RUNTIME_VERSION(RuntimeVersionProperty.class, config -> config.runtimeVersion),

  /** Directive for specifying a specific runtime scheduler, if supported. */
  SCHEDULER(SchedulerProperty.class, config -> config.schedulerType),
  /** Directive to specify that all code is generated in a single file. */
  SINGLE_FILE_PROJECT(SingleFileProjectProperty.class, config -> config.singleFileProject),

  /** Directive to indicate whether the runtime should use multi-threading. */
  THREADING(ThreadingProperty.class, config -> config.threading),
  /** Directive to check the generated verification model. */
  VERIFY(VerifyProperty.class, config -> config.verify),

  /** Directive to specify the number of worker threads used by the runtime. */
  WORKERS(WorkersProperty.class, config -> config.workers),

  /** Directive to specify the execution timeout. */
  TIMEOUT(TimeOutProperty.class, config -> config.timeout),

  /** Directive to enable tracing. */
  TRACING(TracingProperty.class, config -> config.tracing),

  /**
   * Directive to let the runtime export its internal dependency graph.
   *
   * <p>This is a debugging feature and currently only used for C++ and Rust programs.
   */
  EXPORT_DEPENDENCY_GRAPH(
      ExportDependencyGraphProperty.class, config -> config.exportDependencyGraph),

  /**
   * Directive to let the runtime export the program structure to a yaml file.
   *
   * <p>This is a debugging feature and currently only used for C++ programs.
   */
  EXPORT_TO_YAML(ExportToYamlProperty.class, config -> config.exportToYaml),

  /**
   * List of module files to link into the crate as top-level. For instance, a {@code target Rust {
   * rust-modules: [ "foo.rs" ] }} will cause the file to be copied into the generated project, and
   * the generated {@code main.rs} will include it with a {@code mod foo;}. If one of the paths is a
   * directory, it must contain a {@code mod.rs} file, and all its contents are copied.
   */
  RUST_INCLUDE(RustIncludeProperty.class, config -> config.rust.rustTopLevelModules),

  /** Directive for specifying Cargo features of the generated program to enable. */
  CARGO_FEATURES(CargoFeaturesProperty.class, config -> config.rust.cargoFeatures),

  /**
   * Dependency specifications for Cargo. This property looks like this:
   *
   * <pre>{@code
   * cargo-dependencies: {
   *    // Name-of-the-crate: "version"
   *    rand: "0.8",
   *    // Equivalent to using an explicit map:
   *    rand: {
   *      version: "0.8"
   *    },
   *    // The map allows specifying more details
   *    rand: {
   *      // A path to a local unpublished crate.
   *      // Note 'path' is mutually exclusive with 'version'.
   *      path: "/home/me/Git/local-rand-clone"
   *    },
   *    rand: {
   *      version: "0.8",
   *      // you can specify cargo features
   *      features: ["some-cargo-feature",]
   *    }
   * }
   * }</pre>
   */
  CARGO_DEPENDENCIES(CargoDependenciesProperty.class, config -> config.rust.cargoDependencies),

  /**
   * Directs the C or Python target to include the associated C file used for setting up federated
   * execution before processing the first tag.
   */
  FED_SETUP(FedSetupProperty.class, config -> config.fedSetupPreamble);

  public final ConfigLoader property;

  public final Class<? extends AbstractTargetProperty<?>> propertyClass;

  @FunctionalInterface
  private interface ConfigLoader {
    AbstractTargetProperty<?> of(TargetConfig config);
  }

  TargetProperty(Class<? extends AbstractTargetProperty<?>> cls, ConfigLoader property) {
    this.propertyClass = cls;
    this.property = property;
  }

  /**
   * Return key ot the property as it will be used in LF code.
   *
   * <p>Keys are of the form <code>foo-bar</code>.
   *
   * @return the property's key
   */
  public String getKey() {
    return name().toLowerCase().replace('_', '-');
  }

  public static AbstractTargetProperty<?> getPropertyInstance(TargetProperty p) {
    try {
      return p.propertyClass.getDeclaredConstructor().newInstance();
    } catch (ReflectiveOperationException e) {
      throw new RuntimeException(e);
    }
  }

  public static void load(TargetConfig config, Properties properties, MessageReporter err) {
    for (Object key : properties.keySet()) {
      TargetProperty p = forName(key.toString());
      if (p != null) {
        try {
          p.property.of(config).set(properties.get(key).toString(), err);
        } catch (InvalidLfSourceException e) {
          err.at(e.getNode()).error(e.getProblem());
        }
      }
    }
  }

  /**
   * Set the given configuration using the given target properties.
   *
   * @param config The configuration object to update.
   * @param properties AST node that holds all the target properties.
   * @param err Error reporter on which property format errors will be reported
   */
  public static void load(TargetConfig config, List<KeyValuePair> properties, MessageReporter err) {
    if (properties == null) {
      return;
    }
    properties.forEach(
        property -> {
          TargetProperty p = forName(property.getName());
          if (p != null) {
            try {
              p.property.of(config).set(property.getValue(), err);
            } catch (InvalidLfSourceException e) {
              err.at(e.getNode()).error(e.getProblem());
            }
          }
        });
  }

  /**
   * Extracts all properties as a list of key-value pairs from a TargetConfig. Only extracts
   * properties explicitly set by user.
   *
   * @param config The TargetConfig to extract from.
   * @return The extracted properties.
   */
  public static List<KeyValuePair> extractProperties(TargetConfig config) {
    var res = new LinkedList<KeyValuePair>();
    for (TargetProperty p : TargetProperty.loaded(config)) {
      KeyValuePair kv = LfFactory.eINSTANCE.createKeyValuePair();
      kv.setName(p.toString());
      kv.setValue(p.property.of(config).toAstElement());
      if (kv.getValue() != null) {
        res.add(kv);
      }
    }
    return res;
  }

  /**
   * Return all the target properties that have been set.
   *
   * @param config The configuration to find the properties in.
   */
  public static List<TargetProperty> loaded(TargetConfig config) {
    return Arrays.stream(TargetProperty.values())
        .filter(it -> it.property.of(config).isSet())
        .collect(Collectors.toList());
  }

  /**
   * Constructs a {@code TargetDecl} by extracting the fields of the given {@code TargetConfig}.
   *
   * @param target The target to generate for.
   * @param config The TargetConfig to extract from.
   * @return A generated TargetDecl.
   */
  public static TargetDecl extractTargetDecl(Target target, TargetConfig config) {
    TargetDecl decl = LfFactory.eINSTANCE.createTargetDecl();
    KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
    for (KeyValuePair p : extractProperties(config)) {
      kvp.getPairs().add(p);
    }
    decl.setName(target.toString());
    decl.setConfig(kvp);
    return decl;
  }

  /**
   * Retrieve a key-value pair from the given AST that matches the given target property.
   *
   * @param ast The AST retrieve the key-value pair from.
   * @param property The target property of interest.
   * @return The found key-value pair, or {@code null} if no matching pair could be found.
   */
  public static KeyValuePair getKeyValuePair(Model ast, TargetProperty property) {
    var targetProperties = ast.getTarget().getConfig();
    List<KeyValuePair> properties =
        targetProperties.getPairs().stream()
            .filter(pair -> pair.getName().equals(property.toString()))
            .toList();
    assert properties.size() <= 1;
    return properties.size() > 0 ? properties.get(0) : null;
  }

  /** Return a list containing the keys of all properties */
  public static List<String> getPropertyKeys() {
    return Arrays.stream(TargetProperty.values())
        .map(TargetProperty::toString)
        .filter(it -> !it.startsWith("_"))
        .sorted()
        .toList();
  }

  /**
   * Validate the given key-value pairs and report issues via the given reporter.
   *
   * @param pairs The key-value pairs to validate.
   * @param ast The root node of the AST from which the key-value pairs were taken.
   * @param config A target configuration used to retrieve the corresponding target properties.
   * @param reporter A reporter to report errors and warnings through.
   */
  public static void validate(
      KeyValuePairs pairs, Model ast, TargetConfig config, ValidatorMessageReporter reporter) {
    pairs.getPairs().stream()
        .forEach(
            pair -> {
              var match =
                  Arrays.stream(TargetProperty.values())
                      .filter(prop -> prop.toString().equalsIgnoreCase(pair.getName()))
                      .findAny();
              if (match.isPresent()) {
                var p = match.get();
                p.property.of(config).checkSupport(pair, config.target, reporter);
                p.property.of(config).checkType(pair, reporter);
                p.property.of(config).validate(pair, ast, reporter);
              } else {
                reporter
                    .at(pair, Literals.KEY_VALUE_PAIR__NAME)
                    .warning(
                        "Unrecognized target property: "
                            + pair.getName()
                            + ". Recognized properties are: "
                            + getPropertyKeys());
              }
            });
  }

  /**
   * Update the given configuration using the given target properties.
   *
   * @param config The configuration object to update.
   * @param properties AST node that holds all the target properties.
   * @param relativePath The path from the main resource to the resource from which the new
   *     properties originate.
   */
  public static void update(
      TargetConfig config, List<KeyValuePair> properties, Path relativePath, MessageReporter err) {
    properties.forEach(
        property -> {
          TargetProperty p = forName(property.getName());
          if (p != null) {
            var value = property.getValue();
            if (property.getName().equals("files")) {
              var array = LfFactory.eINSTANCE.createArray();
              ASTUtils.elementToListOfStrings(property.getValue()).stream()
                  .map(relativePath::resolve) // assume all paths are relative
                  .map(Objects::toString)
                  .map(
                      s -> {
                        var element = LfFactory.eINSTANCE.createElement();
                        element.setLiteral(s);
                        return element;
                      })
                  .forEach(array.getElements()::add);
              value = LfFactory.eINSTANCE.createElement();
              value.setArray(array);
            }
            p.property.of(config).set(value, err);
          }
        });
  }

  /**
   * Return the entry that matches the given string.
   *
   * @param name The string to match against.
   */
  public static TargetProperty forName(String name) {
    return Target.match(name, TargetProperty.values());
  }

  /**
   * Return a list with all target properties.
   *
   * @return All existing target properties.
   */
  public static List<TargetProperty> getOptions() {
    return Arrays.asList(TargetProperty.values());
  }

  /**
   * Return the name of the property in as it appears in the target declaration. It may be an
   * invalid identifier in other languages (may contains dashes {@code -}).
   */
  @Override
  public String toString() {
    // Workaround because this sole property does not follow the naming convention.
    if (this.equals(FED_SETUP)) {
      return "_fed_setup";
    }
    return this.name().toLowerCase().replaceAll("_", "-");
  }
}
