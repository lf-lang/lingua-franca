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

import org.lflang.ast.ASTUtils;
import org.lflang.generator.InvalidLfSourceException;
import org.lflang.generator.rust.CargoDependencySpec;
import org.lflang.generator.rust.CargoDependencySpec.CargoDependenciesPropertyType;
import org.lflang.lf.Array;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Model;
import org.lflang.lf.TargetDecl;
import org.lflang.target.LoggingConfigurator.LogLevel;
import org.lflang.target.property.type.ArrayType;
import org.lflang.target.property.type.DictionaryType;
import org.lflang.target.property.type.StringDictionaryType;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.target.property.type.PrimitiveType;
import org.lflang.target.property.type.UnionType;
import org.lflang.util.FileUtil;
import org.lflang.util.StringUtil;
import org.lflang.validation.ValidationReporter;

/**
 * A target properties along with a type and a list of supporting targets that supports it, as well
 * as a function for configuration updates.
 *
 * @author Marten Lohstroh
 */
public enum TargetProperty {
  /** Directive to allow including OpenSSL libraries and process HMAC authentication. */
      AUTH(
          "auth",
          PrimitiveType.BOOLEAN,
          Arrays.asList(Target.C, Target.CCPP),
          (TargetConfig config) -> config.auth),

      /** Directive to let the generator use the custom build command. */
      BUILD(
          "build",
          UnionType.STRING_OR_STRING_ARRAY,
          Arrays.asList(Target.C, Target.CCPP),
          (TargetConfig config) -> config.buildCommands),

      /**
       * Directive to specify the target build type such as 'Release' or 'Debug'. This is also used in
       * the Rust target to select a Cargo profile.
       */
      BUILD_TYPE(
          "build-type",
          UnionType.BUILD_TYPE_UNION,
          Arrays.asList(Target.C, Target.CCPP, Target.CPP, Target.Rust),
          (TargetConfig config) -> config.buildType),

      /** Directive to let the federate execution handle clock synchronization in software. */
      CLOCK_SYNC(
          "clock-sync",
          UnionType.CLOCK_SYNC_UNION,
          Arrays.asList(Target.C, Target.CCPP, Target.Python),
          (TargetConfig config) -> config.clockSync),
      /** Key-value pairs giving options for clock synchronization. */
      CLOCK_SYNC_OPTIONS(
          "clock-sync-options",
          DictionaryType.CLOCK_SYNC_OPTION_DICT,
          Arrays.asList(Target.C, Target.CCPP, Target.Python),
          (TargetConfig config) -> config.clockSyncOptions),

      /**
       * Directive to specify a cmake to be included by the generated build systems.
       *
       * <p>This gives full control over the C/C++ build as any cmake parameters can be adjusted in the
       * included file.
       */
      CMAKE_INCLUDE(
          "cmake-include",
          UnionType.FILE_OR_FILE_ARRAY,
          Arrays.asList(Target.CPP, Target.C, Target.CCPP),
          (TargetConfig config) -> config.cmakeIncludes),
      /** Directive to specify the target compiler. */
      COMPILER(
          "compiler",
          PrimitiveType.STRING,
          Target.ALL,
          (config) -> ASTUtils.toElement(config.compiler),
          (config, value, err) -> {
              config.compiler = ASTUtils.elementToSingleString(value);
          }),

      /** Directive to specify compile-time definitions. */
      COMPILE_DEFINITIONS(
          "compile-definitions",
          StringDictionaryType.COMPILE_DEFINITION,
          Arrays.asList(Target.C, Target.CCPP, Target.Python),
          (config) -> ASTUtils.toElement(config.compileDefinitions),
          (config, value, err) -> {
              config.compileDefinitions = ASTUtils.elementToStringMaps(value);
          }),

      /**
       * Directive to generate a Dockerfile. This is either a boolean, true or false, or a dictionary of
       * options.
       */
      DOCKER(
          "docker",
          UnionType.DOCKER_UNION,
          Arrays.asList(Target.C, Target.CCPP, Target.Python, Target.TS),
          (TargetConfig config) -> config.dockerOptions),

      /** Directive for specifying a path to an external runtime to be used for the compiled binary. */
      EXTERNAL_RUNTIME_PATH(
          "external-runtime-path",
          PrimitiveType.STRING,
          List.of(Target.CPP),
          (config) -> ASTUtils.toElement(config.externalRuntimePath),
          (config, value, err) -> {
              config.externalRuntimePath = ASTUtils.elementToSingleString(value);
          }),

      /**
       * Directive to let the execution engine allow logical time to elapse faster than physical time.
       */
      FAST(
          "fast",
          PrimitiveType.BOOLEAN,
          Target.ALL,
          (TargetConfig config) -> config.fastMode),
      /**
       * Directive to stage particular files on the class path to be processed by the code generator.
       */
      FILES(
          "files",
          UnionType.FILE_OR_FILE_ARRAY,
          List.of(Target.C, Target.CCPP, Target.Python),
          (config) -> ASTUtils.toElement(config.files),
          (config, value, err) -> {
              config.files = ASTUtils.elementToListOfStrings(value);
          },
          (config, value, err) -> {
              config.files.addAll(ASTUtils.elementToListOfStrings(value));
          }),

      /** Flags to be passed on to the target compiler. */
      FLAGS(
          "flags",
          UnionType.STRING_OR_STRING_ARRAY,
          Arrays.asList(Target.C, Target.CCPP),
          (config) -> ASTUtils.toElement(config.compilerFlags),
          (config, value, err) -> {
              config.compilerFlags = ASTUtils.elementToListOfStrings(value);
          }),

      /** Directive to specify the coordination mode */
      COORDINATION(
          "coordination",
          UnionType.COORDINATION_UNION,
          Arrays.asList(Target.C, Target.CCPP, Target.Python),
          (TargetConfig config) -> config.coordination),
      /** Key-value pairs giving options for clock synchronization. */
      COORDINATION_OPTIONS(
          "coordination-options",
          DictionaryType.COORDINATION_OPTION_DICT,
          Arrays.asList(Target.C, Target.CCPP, Target.Python, Target.TS),
          (TargetConfig config) -> config.coordinationOptions),

      /**
       * Directive to let the execution engine remain active also if there are no more events in the
       * event queue.
       */
      KEEPALIVE(
          "keepalive",
          PrimitiveType.BOOLEAN,
          Target.ALL,
          (TargetConfig config) -> config.keepalive),

      /** Directive to specify the grain at which to report log messages during execution. */
      LOGGING(
          "logging",
          UnionType.LOGGING_UNION,
          Target.ALL,
          (config) -> ASTUtils.toElement(config.logLevel.toString()),
          (config, value, err) -> {
              config.logLevel =
                  (LogLevel) UnionType.LOGGING_UNION.forName(ASTUtils.elementToSingleString(value));
          }),

      /** Directive to not invoke the target compiler. */
      NO_COMPILE(
          "no-compile",
          PrimitiveType.BOOLEAN,
          Arrays.asList(Target.C, Target.CPP, Target.CCPP, Target.Python),
          (config) -> ASTUtils.toElement(config.noCompile),
          (config, value, err) -> {
              config.noCompile = ASTUtils.toBoolean(value);
          }),

      /** Directive to disable validation of reactor rules at runtime. */
      NO_RUNTIME_VALIDATION(
          "no-runtime-validation",
          PrimitiveType.BOOLEAN,
          Arrays.asList(Target.CPP),
          (config) -> ASTUtils.toElement(config.noRuntimeValidation),
          (config, value, err) -> {
              config.noRuntimeValidation = ASTUtils.toBoolean(value);
          }),

      /** Directive to check the generated verification model. */
      VERIFY(
          "verify",
          PrimitiveType.BOOLEAN,
          Arrays.asList(Target.C),
          (config) -> ASTUtils.toElement(config.verify),
          (config, value, err) -> {
              config.verify = ASTUtils.toBoolean(value);
          }),

      /**
       * Directive to specify the platform for cross code generation. This is either a string of the
       * platform or a dictionary of options that includes the string name.
       */
      PLATFORM(
          "platform",
          UnionType.PLATFORM_STRING_OR_DICTIONARY,
          Target.ALL,
          (TargetConfig config) -> config.platformOptions),

      /** Directive to instruct the runtime to collect and print execution statistics. */
      PRINT_STATISTICS(
          "print-statistics",
          PrimitiveType.BOOLEAN,
          Arrays.asList(Target.CPP),
          (config) -> ASTUtils.toElement(config.printStatistics),
          (config, value, err) -> {
              config.printStatistics = ASTUtils.toBoolean(value);
          }),

      /**
       * Directive for specifying .proto files that need to be compiled and their code included in the
       * sources.
       */
      PROTOBUFS(
          "protobufs",
          UnionType.FILE_OR_FILE_ARRAY,
          Arrays.asList(Target.C, Target.CCPP, Target.TS, Target.Python),
          (config) -> ASTUtils.toElement(config.protoFiles),
          (config, value, err) -> {
              config.protoFiles = ASTUtils.elementToListOfStrings(value);
          }),

      /** Directive to specify that ROS2 specific code is generated, */
      ROS2(
          "ros2",
          PrimitiveType.BOOLEAN,
          List.of(Target.CPP),
          (config) -> ASTUtils.toElement(config.ros2),
          (config, value, err) -> {
              config.ros2 = ASTUtils.toBoolean(value);
          }),

      /** Directive to specify additional ROS2 packages that this LF program depends on. */
      ROS2_DEPENDENCIES(
          "ros2-dependencies",
          ArrayType.STRING_ARRAY,
          List.of(Target.CPP),
          (TargetConfig config) -> config.platformOptions),

      /** Directive for specifying a specific version of the reactor runtime library. */
      RUNTIME_VERSION(
          "runtime-version",
          PrimitiveType.STRING,
          Arrays.asList(Target.CPP),
          (config) -> ASTUtils.toElement(config.runtimeVersion),
          (config, value, err) -> {
              config.runtimeVersion = ASTUtils.elementToSingleString(value);
          }),

      /** Directive for specifying a specific runtime scheduler, if supported. */
      SCHEDULER(
          "scheduler",
          UnionType.SCHEDULER_UNION,
          Arrays.asList(Target.C, Target.CCPP, Target.Python),
          (TargetConfig config) -> config.schedulerType
          ),
      /** Directive to specify that all code is generated in a single file. */
      SINGLE_FILE_PROJECT(
          "single-file-project",
          PrimitiveType.BOOLEAN,
          List.of(Target.Rust),
          (config) -> ASTUtils.toElement(config.singleFileProject),
          (config, value, err) -> {
              config.singleFileProject = ASTUtils.toBoolean(value);
          }),

      /** Directive to indicate whether the runtime should use multi-threading. */
      THREADING(
          "threading",
          PrimitiveType.BOOLEAN,
          List.of(Target.C, Target.CCPP, Target.Python),
          (config) -> ASTUtils.toElement(config.threading),
          (config, value, err) -> {
              config.threading = ASTUtils.toBoolean(value);
          }),

      /** Directive to specify the number of worker threads used by the runtime. */
      WORKERS(
          "workers",
          PrimitiveType.NON_NEGATIVE_INTEGER,
          List.of(Target.C, Target.CCPP, Target.Python, Target.CPP, Target.Rust),
          (config) -> ASTUtils.toElement(config.workers),
          (config, value, err) -> {
              config.workers = ASTUtils.toInteger(value);
          }),

      /** Directive to specify the execution timeout. */
      TIMEOUT(
          "timeout",
          PrimitiveType.TIME_VALUE,
          Target.ALL,
          (config) -> ASTUtils.toElement(config.timeout),
          (config, value, err) -> {
              config.timeout = ASTUtils.toTimeValue(value);
          },
          (config, value, err) -> {
              config.timeout = ASTUtils.toTimeValue(value);
          }),

      /** Directive to enable tracing. */
      TRACING(
          "tracing",
          UnionType.TRACING_UNION,
          Arrays.asList(Target.C, Target.CCPP, Target.CPP, Target.Python),
          (TargetConfig config) -> config.tracing),

      /**
       * Directive to let the runtime export its internal dependency graph.
       *
       * <p>This is a debugging feature and currently only used for C++ and Rust programs.
       */
      EXPORT_DEPENDENCY_GRAPH(
          "export-dependency-graph",
          PrimitiveType.BOOLEAN,
          List.of(Target.CPP, Target.Rust),
          (config) -> ASTUtils.toElement(config.exportDependencyGraph),
          (config, value, err) -> {
              config.exportDependencyGraph = ASTUtils.toBoolean(value);
          }),

      /**
       * Directive to let the runtime export the program structure to a yaml file.
       *
       * <p>This is a debugging feature and currently only used for C++ programs.
       */
      EXPORT_TO_YAML(
          "export-to-yaml",
          PrimitiveType.BOOLEAN,
          List.of(Target.CPP),
          (config) -> ASTUtils.toElement(config.exportToYaml),
          (config, value, err) -> {
              config.exportToYaml = ASTUtils.toBoolean(value);
          }),

      /**
       * List of module files to link into the crate as top-level. For instance, a {@code target Rust {
       * rust-modules: [ "foo.rs" ] }} will cause the file to be copied into the generated project, and
       * the generated {@code main.rs} will include it with a {@code mod foo;}. If one of the paths is a
       * directory, it must contain a {@code mod.rs} file, and all its contents are copied.
       */
      RUST_INCLUDE(
          "rust-include",
          UnionType.FILE_OR_FILE_ARRAY,
          List.of(Target.Rust),
          (config) -> {
              // do not check paths here, and simply copy the absolute path over
              List<Path> paths = config.rust.getRustTopLevelModules();
              if (paths.isEmpty()) {
                  return null;
              } else if (paths.size() == 1) {
                  return ASTUtils.toElement(paths.get(0).toString());
              } else {
                  Element e = LfFactory.eINSTANCE.createElement();
                  Array arr = LfFactory.eINSTANCE.createArray();
                  for (Path p : paths) {
                      arr.getElements().add(ASTUtils.toElement(p.toString()));
                  }
                  e.setArray(arr);
                  return e;
              }
          },
          (config, value, err) -> {
              Path referencePath;
              try {
                  referencePath = FileUtil.toPath(value.eResource().getURI()).toAbsolutePath();
              } catch (IllegalArgumentException e) {
                  err.at(value).error("Invalid path? " + e.getMessage());
                  throw e;
              }

              // we'll resolve relative paths to check that the files
              // are as expected.

              if (value.getLiteral() != null) {
                  Path resolved = referencePath.resolveSibling(StringUtil.removeQuotes(value.getLiteral()));

                  config.rust.addAndCheckTopLevelModule(resolved, value, err);
              } else if (value.getArray() != null) {
                  for (Element element : value.getArray().getElements()) {
                      String literal = StringUtil.removeQuotes(element.getLiteral());
                      Path resolved = referencePath.resolveSibling(literal);
                      config.rust.addAndCheckTopLevelModule(resolved, element, err);
                  }
              }
          }),

      /** Directive for specifying Cargo features of the generated program to enable. */
      CARGO_FEATURES(
          "cargo-features",
          ArrayType.STRING_ARRAY,
          List.of(Target.Rust),
          (config) -> ASTUtils.toElement(config.rust.getCargoFeatures()),
          (config, value, err) -> {
              config.rust.setCargoFeatures(ASTUtils.elementToListOfStrings(value));
          }),

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
      CARGO_DEPENDENCIES(
          "cargo-dependencies",
          CargoDependenciesPropertyType.INSTANCE,
          List.of(Target.Rust),
          (config) -> {
              var deps = config.rust.getCargoDependencies();
              if (deps.size() == 0) {
                  return null;
              } else {
                  Element e = LfFactory.eINSTANCE.createElement();
                  KeyValuePairs kvp = LfFactory.eINSTANCE.createKeyValuePairs();
                  for (var ent : deps.entrySet()) {
                      KeyValuePair pair = LfFactory.eINSTANCE.createKeyValuePair();
                      pair.setName(ent.getKey());
                      pair.setValue(CargoDependencySpec.extractSpec(ent.getValue()));
                      kvp.getPairs().add(pair);
                  }
                  e.setKeyvalue(kvp);
                  return e;
              }
          },
          (config, value, err) -> {
              config.rust.setCargoDependencies(CargoDependencySpec.parseAll(value));
          }),

      /**
       * Directs the C or Python target to include the associated C file used for setting up federated
       * execution before processing the first tag.
       */
      FED_SETUP(
          "_fed_setup",
          PrimitiveType.FILE,
          Arrays.asList(Target.C, Target.CCPP, Target.Python),
          (config) -> ASTUtils.toElement(config.fedSetupPreamble),
          (config, value, err) ->
              config.fedSetupPreamble = StringUtil.removeQuotes(ASTUtils.elementToSingleString(value)));


  /** String representation of this target property. */
  public final String description;

  /**
   * List of targets that support this property. If a property is used for a target that does not
   * support it, a warning reported during validation.
   */
  public final List<Target> supportedBy;

  /** The type of values that can be assigned to this property. */
  public final TargetPropertyType type;

  /**
   * Function that given a configuration object and an Element AST node sets the configuration. It
   * is assumed that validation already occurred, so this code should be straightforward.
   */
  public final PropertyParser setter;

  private final PropertyValidator validator;


    /**
   * Function that given a configuration object and an Element AST node sets the configuration. It
   * is assumed that validation already occurred, so this code should be straightforward.
   */
  public final PropertyParser updater;


  @FunctionalInterface
  private interface ConfigGetter<T> {
      TargetPropertyConfig<T> get(TargetConfig config);
  }

  @FunctionalInterface
  private interface  PropertyValidator {
      void validate(KeyValuePair pair, Model ast, TargetConfig config, ValidationReporter reporter);
  }

  @FunctionalInterface
  private interface PropertyParser {

    /**
     * Parse the given element into the given target config. May use the error reporter to report
     * format errors.
     */
    void parseIntoTargetConfig(TargetConfig config, Element element, MessageReporter err);
  }

  public final PropertyGetter getter;

  @FunctionalInterface
  private interface PropertyGetter {

    /**
     * Read this property from the target config and build an element which represents it for the
     * AST. May return null if the given value of this property is the same as the default.
     */
    Element getPropertyElement(TargetConfig config);
  }

  /**
   * Private constructor for target properties.
   *
   * @param description String representation of this property.
   * @param type The type that values assigned to this property should conform to.
   * @param supportedBy List of targets that support this property.
   * @param setter Function for configuration updates.
   */
  TargetProperty(
      String description,
      TargetPropertyType type,
      List<Target> supportedBy,
      PropertyGetter getter,
      PropertyParser setter) {
    this.description = description;
    this.type = type;
    this.supportedBy = supportedBy;
    this.getter = getter;
    this.setter = setter;
    this.updater = setter; // (Re)set by default
      this.validator = (pair, ast, config, validator) -> {};
  }

  TargetProperty(String description, TargetPropertyType type, List<Target> supportedBy,
      ConfigGetter g) {
      this.description = description;
      this.type = type;
      this.supportedBy = supportedBy;
      this.setter = (TargetConfig config, Element element, MessageReporter err) -> {
          g.get(config).set(element, err);
      };
      this.updater = (TargetConfig config, Element element, MessageReporter err) -> {
          g.get(config).set(element, err);
      };

      this.getter = (TargetConfig config) -> {
          return g.get(config).export();
      };

      this.validator = (KeyValuePair pair, Model ast, TargetConfig config, ValidationReporter reporter) -> {
          g.get(config).validate(pair, ast, config, reporter);
      };
  }

  /**
   * Private constructor for target properties. This will take an additional {@code updater}, which
   * will be used to merge target properties from imported resources.
   *
   * @param description String representation of this property.
   * @param type The type that values assigned to this property should conform to.
   * @param supportedBy List of targets that support this property.
   * @param setter Function for setting configuration values.
   * @param updater Function for updating configuration values.
   */
  TargetProperty(
      String description,
      TargetPropertyType type,
      List<Target> supportedBy,
      PropertyGetter getter,
      PropertyParser setter,
      PropertyParser updater) {
    this.description = description;
    this.type = type;
    this.supportedBy = supportedBy;
    this.getter = getter;
    this.setter = setter;
    this.updater = updater;
      this.validator = (pair, ast, config, validator) -> {};
  }

  /**
   * Return the name of the property in lingua franca. This is suitable for use as a key in a target
   * properties block. It may be an invalid identifier in other languages (may contains dashes
   * {@code -}).
   */
  public String getDisplayName() {
    return description;
  }

  public static void override(TargetConfig config, Properties properties, MessageReporter err) {
      properties.keySet().forEach(
          key -> {
              TargetProperty p = forName(key.toString());
              if (p != null) {
                  // Mark the specified target property as set by the user
                  config.setByUser.add(p);
                  try {
                      // FIXME: p.setter.parseIntoTargetConfig(config, properties.get(key), err);
                  } catch (InvalidLfSourceException e) {
                      err.at(e.getNode()).error(e.getProblem());
                  }
              }
          });
  }

  /**
   * Set the given configuration using the given target properties.
   *
   * @param config The configuration object to update.
   * @param properties AST node that holds all the target properties.
   * @param err Error reporter on which property format errors will be reported
   */
  public static void set(TargetConfig config, List<KeyValuePair> properties, MessageReporter err) {
    properties.forEach(
        property -> {
          TargetProperty p = forName(property.getName());
          if (p != null) {
            // Mark the specified target property as set by the user
            config.setByUser.add(p);
            try {
              p.setter.parseIntoTargetConfig(config, property.getValue(), err);
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
    for (TargetProperty p : config.setByUser) {
      KeyValuePair kv = LfFactory.eINSTANCE.createKeyValuePair();
      kv.setName(p.toString());
      kv.setValue(p.getter.getPropertyElement(config));
      if (kv.getValue() != null) res.add(kv);
    }
    return res;
  }

  /**
   * Constructs a TargetDecl by extracting the fields of the given TargetConfig.
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

    public static KeyValuePair getKeyValuePair(KeyValuePairs targetProperties, TargetProperty property) {
        List<KeyValuePair> properties =
            targetProperties.getPairs().stream()
                .filter(pair -> pair.getName().equals(property.description))
                .toList();
        assert (properties.size() <= 1);
        return properties.size() > 0 ? properties.get(0) : null;
    }

    public static KeyValuePair getKeyValuePair(Model ast, TargetProperty property) {
      return getKeyValuePair(ast.getTarget().getConfig(), property);
    }

    public void validate(KeyValuePairs pairs, Model ast, TargetConfig config, ValidationReporter reporter) {
      this.validator.validate(getKeyValuePair(pairs, this), ast, config, reporter);
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
            // Mark the specified target property as set by the user
            config.setByUser.add(p);
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
            p.updater.parseIntoTargetConfig(config, value, err);
          }
        });
  }

  /**
   * Update one of the target properties, given by 'propertyName'. For convenience, a list of target
   * properties (e.g., taken from a file or resource) can be passed without any filtering. This
   * function will do nothing if the list of target properties doesn't include the property given by
   * 'propertyName'.
   *
   * @param config The target config to apply the update to.
   * @param property The target property.
   * @param properties AST node that holds all the target properties.
   * @param err Error reporter on which property format errors will be reported
   */
  public static void updateOne(
      TargetConfig config,
      TargetProperty property,
      List<KeyValuePair> properties,
      MessageReporter err) {
    properties.stream()
        .filter(p -> p.getName().equals(property.getDisplayName()))
        .findFirst()
        .map(KeyValuePair::getValue)
        .ifPresent(value -> property.updater.parseIntoTargetConfig(config, value, err));
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

  /** Return the description. */
  @Override
  public String toString() {
    return this.description;
  }

  /** Interface for dictionary elements. It associates an entry with a type. */
  public interface DictionaryElement {

    TargetPropertyType getType();
  }
}
