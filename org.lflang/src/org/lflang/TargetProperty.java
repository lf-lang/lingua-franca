/*************
Copyright (c) 2019, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/

package org.lflang;

import java.io.IOException;
import java.nio.file.Path;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import org.eclipse.xtext.util.RuntimeIOException;
import org.lflang.TargetConfig.Board;
import org.lflang.TargetConfig.DockerOptions;
import org.lflang.TargetConfig.PlatformOptions;
import org.lflang.TargetConfig.TracingOptions;
import org.lflang.generator.InvalidLfSourceException;
import org.lflang.generator.rust.CargoDependencySpec;
import org.lflang.generator.rust.CargoDependencySpec.CargoDependenciesPropertyType;
import org.lflang.lf.Array;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.util.FileUtil;
import org.lflang.util.StringUtil;
import org.lflang.validation.LFValidator;

import com.google.common.collect.ImmutableList;

/**
 * A target properties along with a type and a list of supporting targets
 * that supports it, as well as a function for configuration updates.
 *
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
public enum TargetProperty {


    /**
     * Directive to let the generator use the custom build command.
     */
    BUILD("build", UnionType.STRING_OR_STRING_ARRAY,
            Arrays.asList(Target.C, Target.CCPP), (config, value, err) -> {
                config.buildCommands = ASTUtils.elementToListOfStrings(value);
            }),

    /**
     * Directive to specify the target build type such as 'Release' or 'Debug'.
     * This is also used in the Rust target to select a Cargo profile.
     */
    BUILD_TYPE("build-type", UnionType.BUILD_TYPE_UNION,
            Arrays.asList(Target.C, Target.CCPP, Target.CPP, Target.Rust), (config, value, err) -> {
                config.cmakeBuildType = (BuildType) UnionType.BUILD_TYPE_UNION
                        .forName(ASTUtils.elementToSingleString(value));
                // set it there too, because the default is different.
                config.rust.setBuildType(config.cmakeBuildType);
            }),

    /**
     * Directive to let the federate execution handle clock synchronization in software.
     */
    CLOCK_SYNC("clock-sync", UnionType.CLOCK_SYNC_UNION,
               Arrays.asList(Target.C, Target.CCPP, Target.Python), (config, value, err) -> {
        config.clockSync = (ClockSyncMode) UnionType.CLOCK_SYNC_UNION
            .forName(ASTUtils.elementToSingleString(value));
    }),

    /**
     * Key-value pairs giving options for clock synchronization.
     */
    CLOCK_SYNC_OPTIONS("clock-sync-options",
            DictionaryType.CLOCK_SYNC_OPTION_DICT, Arrays.asList(Target.C, Target.CCPP, Target.Python),
            (config, value, err) -> {
                for (KeyValuePair entry : value.getKeyvalue().getPairs()) {
                    ClockSyncOption option = (ClockSyncOption) DictionaryType.CLOCK_SYNC_OPTION_DICT
                            .forName(entry.getName());
                    switch (option) {
                        case ATTENUATION:
                            config.clockSyncOptions.attenuation = ASTUtils
                                    .toInteger(entry.getValue());
                            break;
                        case COLLECT_STATS:
                            config.clockSyncOptions.collectStats = ASTUtils
                                    .toBoolean(entry.getValue());
                            break;
                        case LOCAL_FEDERATES_ON:
                            config.clockSyncOptions.localFederatesOn = ASTUtils
                                    .toBoolean(entry.getValue());
                            break;
                        case PERIOD:
                            config.clockSyncOptions.period = ASTUtils
                                    .toTimeValue(entry.getValue());
                            break;
                        case TEST_OFFSET:
                            config.clockSyncOptions.testOffset = ASTUtils
                                    .toTimeValue(entry.getValue());
                            break;
                        case TRIALS:
                            config.clockSyncOptions.trials = ASTUtils
                                    .toInteger(entry.getValue());
                            break;
                        default:
                            break;
                    }
                }
            }),

    /**
     * Directive to specify a cmake to be included by the generated build
     * systems.
     *
     * This gives full control over the C/C++ build as any cmake parameters
     * can be adjusted in the included file.
     */
    CMAKE_INCLUDE("cmake-include", UnionType.FILE_OR_FILE_ARRAY,
            Arrays.asList(Target.CPP, Target.C, Target.CCPP), (config, value, err) -> {
                config.cmakeIncludes = ASTUtils.elementToListOfStrings(value);
            },
            // FIXME: This merging of lists is potentially dangerous since
            // the incoming list of cmake-includes can belong to a .lf file that is
            // located in a different location, and keeping just filename
            // strings like this without absolute paths is incorrect.
            (config, value, err) -> {
                config.cmakeIncludes.addAll(ASTUtils.elementToListOfStrings(value));
            }),

    /**
     * Directive to specify the target compiler.
     */
    COMPILER("compiler", PrimitiveType.STRING, Target.ALL,
            (config, value, err) -> {
                config.compiler = ASTUtils.elementToSingleString(value);
            }),

    /**
     * Directive to generate a Dockerfile. This is either a boolean,
     * true or false, or a dictionary of options.
     */
    DOCKER("docker", UnionType.DOCKER_UNION,
            Arrays.asList(Target.C, Target.CCPP, Target.Python, Target.TS), (config, value, err) -> {
                if (value.getLiteral() != null) {
                    if (ASTUtils.toBoolean(value)) {
                        config.dockerOptions = new DockerOptions();
                    } else {
                        config.dockerOptions = null;
                    }
                } else {
                    config.dockerOptions = new DockerOptions();
                    for (KeyValuePair entry : value.getKeyvalue().getPairs()) {
                        DockerOption option = (DockerOption) DictionaryType.DOCKER_DICT
                                .forName(entry.getName());
                        switch (option) {
                            case FROM:
                                config.dockerOptions.from = ASTUtils.elementToSingleString(entry.getValue());
                                break;
                            default:
                                break;
                        }
                    }
                }
            }),

    /**
     * Directive for specifying a path to an external runtime to be used for the
     * compiled binary.
     */
    EXTERNAL_RUNTIME_PATH("external-runtime-path", PrimitiveType.STRING,
            List.of(Target.CPP), (config, value, err) -> {
                config.externalRuntimePath = ASTUtils.elementToSingleString(value);
            }),

    /**
     * Directive to let the execution engine allow logical time to elapse
     * faster than physical time.
     */
    FAST("fast", PrimitiveType.BOOLEAN, Target.ALL,
            (config, value, err) -> {
                config.fastMode = ASTUtils.toBoolean(value);
            }),

    /**
     * Directive to stage particular files on the class path to be
     * processed by the code generator.
     */
    FILES("files", UnionType.FILE_OR_FILE_ARRAY, List.of(Target.C, Target.CCPP, Target.Python),
            (config, value, err) -> {
                config.fileNames = ASTUtils.elementToListOfStrings(value);
            },
            // FIXME: This merging of lists is potentially dangerous since
            // the incoming list of files can belong to a .lf file that is
            // located in a different location, and keeping just filename
            // strings like this without absolute paths is incorrect.
            (config, value, err) -> {
                config.fileNames.addAll(ASTUtils.elementToListOfStrings(value));
            }),

    /**
     * Flags to be passed on to the target compiler.
     */
    FLAGS("flags", UnionType.STRING_OR_STRING_ARRAY,
        Arrays.asList(Target.C, Target.CCPP), (config, value, err) -> {
        config.compilerFlags = ASTUtils.elementToListOfStrings(value);
    }),

    /**
     * Directive to specify the coordination mode
     */
    COORDINATION("coordination", UnionType.COORDINATION_UNION,
            Arrays.asList(Target.C, Target.CCPP, Target.Python),
            (config, value, err) -> {
                config.coordination = (CoordinationType) UnionType.COORDINATION_UNION
                        .forName(ASTUtils.elementToSingleString(value));
            }),

    /**
     * Key-value pairs giving options for clock synchronization.
     */
    COORDINATION_OPTIONS("coordination-options",
            DictionaryType.COORDINATION_OPTION_DICT, Arrays.asList(Target.C, Target.CCPP, Target.Python, Target.TS),
            (config, value, err) -> {
                for (KeyValuePair entry : value.getKeyvalue().getPairs()) {
                    CoordinationOption option = (CoordinationOption) DictionaryType.COORDINATION_OPTION_DICT
                            .forName(entry.getName());
                    switch (option) {
                        case ADVANCE_MESSAGE_INTERVAL:
                            config.coordinationOptions.advance_message_interval = ASTUtils
                                    .toTimeValue(entry.getValue());
                            break;
                        default:
                            break;
                    }
                }
            }),

    /**
     * Directive to let the execution engine remain active also if there
     * are no more events in the event queue.
     */
    KEEPALIVE("keepalive", PrimitiveType.BOOLEAN, Target.ALL,
            (config, value, err) -> {
                config.keepalive = ASTUtils.toBoolean(value);
            }),

    /**
     * Directive to specify the grain at which to report log messages during execution.
     */
    LOGGING("logging", UnionType.LOGGING_UNION, Target.ALL,
            (config, value, err) -> {
                config.logLevel = (LogLevel) UnionType.LOGGING_UNION
                        .forName(ASTUtils.elementToSingleString(value));
            }),

    /**
     * Directive to not invoke the target compiler.
     */
    NO_COMPILE("no-compile", PrimitiveType.BOOLEAN,
            Arrays.asList(Target.C, Target.CPP, Target.CCPP, Target.Python),
            (config, value, err) -> {
                config.noCompile = ASTUtils.toBoolean(value);
            }),

    /**
     * Directive to disable validation of reactor rules at runtime.
     */
    NO_RUNTIME_VALIDATION("no-runtime-validation", PrimitiveType.BOOLEAN,
            Arrays.asList(Target.CPP), (config, value, err) -> {
                config.noRuntimeValidation = ASTUtils.toBoolean(value);
            }),

    /**
     * Directive to specify the platform for cross code generation. This is either a string of the platform
     * or a dictionary of options that includes the string name.
     */
    PLATFORM("platform", UnionType.PLATFORM_STRING_OR_DICTIONARY, Target.ALL, 
            (config, value, err) -> {
                if (value.getLiteral() != null) {
                    config.platformOptions = new PlatformOptions();
                    config.platformOptions.platform = (Platform) UnionType.PLATFORM_UNION
                        .forName(ASTUtils.elementToSingleString(value));
                } else {
                    config.platformOptions= new PlatformOptions();
                    for (KeyValuePair entry : value.getKeyvalue().getPairs()) {
                        PlatformOption option = (PlatformOption) DictionaryType.PLATFORM_DICT
                                .forName(entry.getName());
                        switch (option) {
                            case NAME:
                                Platform p = (Platform) UnionType.PLATFORM_UNION
                                    .forName(ASTUtils.elementToSingleString(entry.getValue()));
                                if(p == null){
                                    String s = "Unidentified Platform Type, LF supports the following platform types: " + Arrays.asList(Platform.values()).toString();
                                    err.reportError(s);
                                    throw new AssertionError(s);
                                }
                                config.platformOptions.platform = p;
                                break;
                            case BAUDRATE:
                                config.platformOptions.baudRate = ASTUtils.toInteger(entry.getValue());
                                break;
                            case BOARD:
                                Board b = (Board) UnionType.BOARD_UNION
                                    .forName(ASTUtils.elementToSingleString(entry.getValue()));
                                if(b == null){
                                    String s = "Unidentified Board Type, LF supports the following board types: " + Arrays.asList(Board.values()).toString();
                                    err.reportError(s);
                                    throw new AssertionError(s);
                                }

                                config.platformOptions.board = b;
                                break;
                            default:
                                break;
                        }
                    }
                }
            }),

    /**
     * Directive for specifying .proto files that need to be compiled and their
     * code included in the sources.
     */
    PROTOBUFS("protobufs", UnionType.FILE_OR_FILE_ARRAY,
            Arrays.asList(Target.C, Target.CCPP, Target.TS, Target.Python),
            (config, value, err) -> {
                config.protoFiles = ASTUtils.elementToListOfStrings(value);
            }),


    /**
     * Directive to specify that ROS2 specific code is generated,
     */
    ROS2("ros2", PrimitiveType.BOOLEAN,
         List.of(Target.CPP), (config, value, err) -> {
             config.ros2 = ASTUtils.toBoolean(value);
    }),

    /**
     * Directive to specify additional ROS2 packages that this LF program depends on.
     */
    ROS2_DEPENDENCIES("ros2-dependencies", ArrayType.STRING_ARRAY,
        List.of(Target.CPP), (config, value, err) -> {
            config.ros2Dependencies = ASTUtils.elementToListOfStrings(value);
    }),

    /**
     * Directive for specifying a specific version of the reactor runtime library.
     */
    RUNTIME_VERSION("runtime-version", PrimitiveType.STRING,
            Arrays.asList(Target.CPP), (config, value, err) -> {
                config.runtimeVersion = ASTUtils.elementToSingleString(value);
            }),


    /**
     * Directive for specifying a specific runtime scheduler, if supported.
     */
    SCHEDULER("scheduler", UnionType.SCHEDULER_UNION,
            Arrays.asList(Target.C, Target.CCPP, Target.Python), (config, value, err) -> {
                config.schedulerType = (SchedulerOption) UnionType.SCHEDULER_UNION
                        .forName(ASTUtils.elementToSingleString(value));
            }),

    /**
     * Directive to specify that all code is generated in a single file.
     */
    SINGLE_FILE_PROJECT("single-file-project", PrimitiveType.BOOLEAN,
            List.of(Target.Rust), (config, value, err) -> {
                config.singleFileProject = ASTUtils.toBoolean(value);
            }),

    /**
     * Directive to indicate whether the runtime should use multi-threading.
     */
    THREADING("threading", PrimitiveType.BOOLEAN,
              List.of(Target.C, Target.CCPP, Target.Python),
              (config, value, err) -> {
                  config.threading = ASTUtils.toBoolean(value);
              }),

    /**
     * Directive to specify the number of worker threads used by the runtime.
     */
    WORKERS("workers", PrimitiveType.NON_NEGATIVE_INTEGER,
            List.of(Target.C, Target.CCPP, Target.Python, Target.CPP, Target.Rust),
            (config, value, err) -> {
                config.workers = ASTUtils.toInteger(value);
            }),

    /**
     * Directive to specify the execution timeout.
     */
    TIMEOUT("timeout", PrimitiveType.TIME_VALUE, Target.ALL,
            (config, value, err) -> {
                config.timeout = ASTUtils.toTimeValue(value);
            }),

    /**
     * Directive to generate a Dockerfile. This is either a boolean,
     * true or false, or a dictionary of options.
     */
    TRACING("tracing", UnionType.TRACING_UNION,
            Arrays.asList(Target.C, Target.CCPP, Target.CPP, Target.Python), (config, value, err) -> {
                if (value.getLiteral() != null) {
                    if (ASTUtils.toBoolean(value)) {
                        config.tracing = new TracingOptions();
                    } else {
                        config.tracing = null;
                    }
                } else {
                    config.tracing = new TracingOptions();
                    for (KeyValuePair entry : value.getKeyvalue().getPairs()) {
                        TracingOption option = (TracingOption) DictionaryType.TRACING_DICT
                            .forName(entry.getName());
                        switch (option) {
                        case TRACE_FILE_NAME:
                            config.tracing.traceFileName = ASTUtils.elementToSingleString(entry.getValue());
                            break;
                        default:
                            break;
                        }
                    }
                }
            }),


    /**
     * Directive to let the runtime export its internal dependency graph.
     *
     * This is a debugging feature and currently only used for C++ and Rust programs.
     */
    EXPORT_DEPENDENCY_GAPH("export-dependency-graph", PrimitiveType.BOOLEAN,
                           List.of(Target.CPP, Target.Rust),
                           (config, value, err) -> {
        config.exportDependencyGraph = ASTUtils.toBoolean(value);
    }),

    /**
     * Directive to let the runtime export the program structure to a yaml file.
     *
     * This is a debugging feature and currently only used for C++ programs.
     */
    EXPORT_TO_YAML("export-to-yaml", PrimitiveType.BOOLEAN,
                           List.of(Target.CPP),
                           (config, value, err) -> {
                               config.exportToYaml = ASTUtils.toBoolean(value);
                           }),

    /**
     * List of module files to link into the crate as top-level.
     * For instance, a {@code target Rust { rust-modules: [ "foo.rs" ] }}
     * will cause the file to be copied into the generated project,
     * and the generated `main.rs` will include it with a `mod foo;`.
     * If one of the paths is a directory, it must contain a `mod.rs`
     * file, and all its contents are copied.
     */
    RUST_INCLUDE("rust-include",
                 UnionType.FILE_OR_FILE_ARRAY,
                 List.of(Target.Rust), (config, value, err) -> {
        Path referencePath;
        try {
            referencePath = FileUtil.toPath(value.eResource().getURI()).toAbsolutePath();
        } catch (IOException e) {
            err.reportError(value, "Invalid path? " + e.getMessage());
            throw new RuntimeIOException(e);
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

    /**
     * Directive for specifying Cargo features of the generated
     * program to enable.
     */
    CARGO_FEATURES("cargo-features", ArrayType.STRING_ARRAY,
                   List.of(Target.Rust), (config, value, err) -> {
        config.rust.setCargoFeatures(ASTUtils.elementToListOfStrings(value));
    }),

    /**
     * Dependency specifications for Cargo. This property looks like this:
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
    CARGO_DEPENDENCIES("cargo-dependencies",
                       CargoDependenciesPropertyType.INSTANCE,
                       List.of(Target.Rust), (config, value, err) -> {
        config.rust.setCargoDependencies(CargoDependencySpec.parseAll(value));
    }),

    ;

    /**
     * String representation of this target property.
     */
    public final String description;

    /**
     * List of targets that support this property. If a property is used for
     * a target that does not support it, a warning reported during
     * validation.
     */
    public final List<Target> supportedBy;

    /**
     * The type of values that can be assigned to this property.
     */
    public final TargetPropertyType type;

    /**
     * Function that given a configuration object and an Element AST node
     * sets the configuration. It is assumed that validation already
     * occurred, so this code should be straightforward.
     */
    public final PropertyParser setter;

    /**
     * Function that given a configuration object and an Element AST node
     * sets the configuration. It is assumed that validation already
     * occurred, so this code should be straightforward.
     */
    public final PropertyParser updater;

    @FunctionalInterface
    private interface PropertyParser {

        /**
         * Parse the given element into the given target config.
         * May use the error reporter to report format errors.
         */
        void parseIntoTargetConfig(TargetConfig config, Element element, ErrorReporter err);
    }

    /**
     * Private constructor for target properties.
     *
     * @param description String representation of this property.
     * @param type        The type that values assigned to this property
     *                    should conform to.
     * @param supportedBy List of targets that support this property.
     * @param setter      Function for configuration updates.
     */
    TargetProperty(String description, TargetPropertyType type,
                   List<Target> supportedBy,
                   PropertyParser setter) {
        this.description = description;
        this.type = type;
        this.supportedBy = supportedBy;
        this.setter = setter;
        this.updater = (config, value, err) -> { /* Ignore the update by default */ };
    }

    /**
     * Private constructor for target properties. This will take an additional
     * `updater`, which will be used to merge target properties from imported resources.
     *
     * @param description String representation of this property.
     * @param type        The type that values assigned to this property
     *                    should conform to.
     * @param supportedBy List of targets that support this property.
     * @param setter      Function for setting configuration values.
     * @param updater     Function for updating configuration values.
     */
    TargetProperty(String description, TargetPropertyType type,
                   List<Target> supportedBy,
                   PropertyParser setter,
                   PropertyParser updater) {
        this.description = description;
        this.type = type;
        this.supportedBy = supportedBy;
        this.setter = setter;
        this.updater = updater;
    }

    /**
     * Return the name of the property in lingua franca. This
     * is suitable for use as a key in a target properties block.
     * It may be an invalid identifier in other languages (may
     * contains dashes {@code -}).
     */
    public String getDisplayName() {
        return description;
    }

    /**
     * Set the given configuration using the given target properties.
     *
     * @param config     The configuration object to update.
     * @param properties AST node that holds all the target properties.
     * @param err        Error reporter on which property format errors will be reported
     */
    public static void set(TargetConfig config, List<KeyValuePair> properties, ErrorReporter err) {
        properties.forEach(property ->  {
            TargetProperty p = forName(property.getName());
            if (p != null) {
                // Mark the specified target property as set by the user
                config.setByUser.add(p);
                try {
                    p.setter.parseIntoTargetConfig(config, property.getValue(), err);
                } catch (InvalidLfSourceException e) {
                    err.reportError(e.getNode(), e.getProblem());
                }
            }
        });
    }

    /**
     * Update the given configuration using the given target properties.
     *
     * @param config     The configuration object to update.
     * @param properties AST node that holds all the target properties.
     */
    public static void update(TargetConfig config, List<KeyValuePair> properties,ErrorReporter err) {
        properties.forEach(property ->  {
            TargetProperty p = forName(property.getName());
            if (p != null) {
                p.updater.parseIntoTargetConfig(config, property.getValue(), err);
            }
        });
    }

    /**
     * Update one of the target properties, given by 'propertyName'.
     * For convenience, a list of target properties (e.g., taken from
     * a file or resource) can be passed without any filtering. This
     * function will do nothing if the list of target properties doesn't
     * include the property given by 'propertyName'.
     *
     * @param config     The target config to apply the update to.
     * @param property   The target property.
     * @param properties AST node that holds all the target properties.
     * @param err        Error reporter on which property format errors will be reported
     */
    public static void updateOne(TargetConfig config, TargetProperty property, List<KeyValuePair> properties, ErrorReporter err) {
        properties.stream()
            .filter(p -> p.getName().equals(property.getDisplayName()))
            .findFirst()
            .map(KeyValuePair::getValue)
            .ifPresent(value -> property.updater.parseIntoTargetConfig(
                config,
                value,
                err
            ));
    }

    /**
     * Return the entry that matches the given string.
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
     * Return the description.
     */
    @Override
    public String toString() {
        return this.description;
    }

    // Inner classes for the various supported types.



    /**
     * Interface for dictionary elements. It associates an entry with a type.
     */
    public interface DictionaryElement {

        TargetPropertyType getType();
    }

    /**
     * A dictionary type with a predefined set of possible keys and assignable
     * types.
     *
     * @author {Marten Lohstroh <marten@berkeley.edu>}
     *
     */
    public enum DictionaryType implements TargetPropertyType {
        CLOCK_SYNC_OPTION_DICT(Arrays.asList(ClockSyncOption.values())),
        DOCKER_DICT(Arrays.asList(DockerOption.values())),
        PLATFORM_DICT(Arrays.asList(PlatformOption.values())),
        COORDINATION_OPTION_DICT(Arrays.asList(CoordinationOption.values())),
        TRACING_DICT(Arrays.asList(TracingOption.values()));

        /**
         * The keys and assignable types that are allowed in this dictionary.
         */
        public List<DictionaryElement> options;

        /**
         * A dictionary type restricted to sets of predefined keys and types of
         * values.
         *
         * @param options The dictionary elements allowed by this type.
         */
        private DictionaryType(List<DictionaryElement> options) {
            this.options = options;
        }

        /**
         * Return the dictionary element of which the key matches the given
         * string.
         *
         * @param name The string to match against.
         * @return The matching dictionary element (or null if there is none).
         */
        public DictionaryElement forName(String name) {
            return Target.match(name, options);
        }

        /**
         * Recursively check that the passed in element conforms to the rules of
         * this dictionary.
         */
        @Override
        public void check(Element e, String name, LFValidator v) {
            KeyValuePairs kv = e.getKeyvalue();
            if (kv == null) {
                TargetPropertyType.produceError(name, this.toString(), v);
            } else {
                for (KeyValuePair pair : kv.getPairs()) {
                    String key = pair.getName();
                    Element val = pair.getValue();
                    Optional<DictionaryElement> match = this.options.stream()
                            .filter(element -> key.equalsIgnoreCase(element.toString())).findAny();
                    if (match.isPresent()) {
                        // Make sure the type is correct, too.
                        TargetPropertyType type = match.get().getType();
                        type.check(val, name + "." + key, v);
                    } else {
                        // No match found; report error.
                        TargetPropertyType.produceError(name,
                                this.toString(), v);
                    }
                }
            }
        }

        /**
         * Return true if the given element represents a dictionary, false
         * otherwise.
         */
        @Override
        public boolean validate(Element e) {
            if (e.getKeyvalue() != null) {
                return true;
            }
            return false;
        }

        /**
         * Return a human-readable description of this type.
         */
        @Override
        public String toString() {
            return "a dictionary with one or more of the following keys: "
                    + options.stream().map(option -> option.toString())
                            .collect(Collectors.joining(", "));
        }
    }
    /**
     * A type that can assume one of several types.
     *
     * @author{Marten Lohstroh <marten@berkeley.edu>}
     *
     */
    public enum UnionType implements TargetPropertyType {
        STRING_OR_STRING_ARRAY(
                Arrays.asList(PrimitiveType.STRING, ArrayType.STRING_ARRAY),
                null),
        PLATFORM_STRING_OR_DICTIONARY(
                Arrays.asList(PrimitiveType.STRING, DictionaryType.PLATFORM_DICT),
                null),
        FILE_OR_FILE_ARRAY(
                Arrays.asList(PrimitiveType.FILE, ArrayType.FILE_ARRAY), null),
        BUILD_TYPE_UNION(Arrays.asList(BuildType.values()), null),
        COORDINATION_UNION(Arrays.asList(CoordinationType.values()),
                CoordinationType.CENTRALIZED),
        SCHEDULER_UNION(Arrays.asList(SchedulerOption.values()), SchedulerOption.getDefault()),
        LOGGING_UNION(Arrays.asList(LogLevel.values()), LogLevel.INFO),
        PLATFORM_UNION(Arrays.asList(Platform.values()), Platform.AUTO),
        BOARD_UNION(Arrays.asList(Board.values()), Board.NONE),
        CLOCK_SYNC_UNION(Arrays.asList(ClockSyncMode.values()),
                ClockSyncMode.INIT),
        DOCKER_UNION(Arrays.asList(PrimitiveType.BOOLEAN, DictionaryType.DOCKER_DICT),
                null),
        TRACING_UNION(Arrays.asList(PrimitiveType.BOOLEAN, DictionaryType.TRACING_DICT),
                null);

        /**
         * The constituents of this type union.
         */
        public final List<Enum<?>> options;

        /**
         * The default type, if there is one.
         */
        private final Enum<?> defaultOption;

        /**
         * Private constructor for creating unions types.
         *
         * @param options The types that that are part of the union.
         * @param defaultOption The default type.
         */
        private UnionType(List<Enum<?>> options, Enum<?> defaultOption) {
            this.options = options;
            this.defaultOption = defaultOption;
        }

        /**
         * Return the type among those in this type union that matches the given
         * name.
         *
         * @param name The string to match against.
         * @return The matching dictionary element (or null if there is none).
         */
        public Enum<?> forName(String name) {
            return Target.match(name, options);
        }

        /**
         * Recursively check that the passed in element conforms to the rules of
         * this union.
         */
        @Override
        public void check(Element e, String name, LFValidator v) {
            Optional<Enum<?>> match = this.match(e);
            if (match.isPresent()) {
                // Go deeper if the element is an array or dictionary.
                Enum<?> type = match.get();
                if (type instanceof DictionaryType) {
                    ((DictionaryType) type).check(e, name, v);
                } else if (type instanceof ArrayType) {
                    ((ArrayType) type).check(e, name, v);
                } else if (type instanceof PrimitiveType) {
                    ((PrimitiveType) type).check(e, name, v);
                } else if (!(type instanceof Enum<?>)) {
                    throw new RuntimeException("Encountered an unknown type.");
                }
            } else {
                // No match found; report error.
                TargetPropertyType.produceError(name, this.toString(), v);
            }
        }

        /**
         * Internal method for matching a given element against the allowable
         * types.
         *
         * @param e AST node that represents the value of a target property.
         * @return The matching type wrapped in an Optional object.
         */
        private Optional<Enum<?>> match(Element e) {
            return this.options.stream().filter(option -> {
                if (option instanceof TargetPropertyType) {
                    return ((TargetPropertyType) option).validate(e);
                } else {
                    return ASTUtils.elementToSingleString(e)
                            .equalsIgnoreCase(option.toString());
                }
            }).findAny();
        }

        /**
         * Return true if this union has an option that matches the given
         * element.
         * @param e The element to match against this type.
         */
        @Override
        public boolean validate(Element e) {
            if (this.match(e).isPresent()) {
                return true;
            }
            return false;
        }

        /**
         * Return a human-readable description of this type. If three is a
         * default option, then indicate it.
         */
        @Override
        public String toString() {
            return "one of the following: " + options.stream().map(option -> {
                if (option == this.defaultOption) {
                    return option.toString() + " (default)";
                } else {
                    return option.toString();
                }
            }).collect(Collectors.joining(", "));
        }

    }

    /**
     * An array type of which the elements confirm to a given type.
     *
     * @author{Marten Lohstroh <marten@berkeley.edu>}
     *
     */
    public enum ArrayType implements TargetPropertyType {
        STRING_ARRAY(PrimitiveType.STRING),
        FILE_ARRAY(PrimitiveType.FILE);

        /**
         * Type parameter of this array type.
         */
        public TargetPropertyType type;

        /**
         * Private constructor to create a new array type.
         *
         * @param type The type of elements in the array.
         */
        private ArrayType(TargetPropertyType type) {
            this.type = type;
        }

        /**
         * Check that the passed in element represents an array and ensure that
         * its elements are all of the correct type.
         */
        @Override
        public void check(Element e, String name, LFValidator v) {
            Array array = e.getArray();
            if (array == null) {
                TargetPropertyType.produceError(name, this.toString(), v);
            } else {
                List<Element> elements = array.getElements();
                for (int i = 0; i < elements.size(); i++) {
                    this.type.check(elements.get(i), name + "[" + i + "]", v);
                }
            }
        }

        /**
         * Return true of the given element is an array.
         */
        @Override
        public boolean validate(Element e) {
            if (e.getArray() != null) {
                return true;
            }
            return false;
        }

        /**
         * Return a human-readable description of this type.
         */
        @Override
        public String toString() {
            return "an array of which each element is " + this.type.toString();
        }
    }

    /**
     * Enumeration of Cmake build types. These are also mapped
     * to Cargo profiles for the Rust target (see {@link org.lflang.generator.rust.RustTargetConfig})
     *
     * @author Christian Menard {@literal <christian.menard@tu-dresden.de>}
     */
    public enum BuildType {
        RELEASE("Release"),
        DEBUG("Debug"),
        TEST("Test"),
        REL_WITH_DEB_INFO("RelWithDebInfo"),
        MIN_SIZE_REL("MinSizeRel");

        /**
         * Alias used in toString method.
         */
        private final String alias;

        /**
         * Private constructor for Cmake build types.
         */
        BuildType(String alias) {
            this.alias = alias;
        }

        /**
         * Return the alias.
         */
        @Override
        public String toString() {
            return this.alias;
        }
    }

    /**
     * Enumeration of coordination types.
     *
     * @author{Marten Lohstroh <marten@berkeley.edu>}
     */
    public enum CoordinationType {
        CENTRALIZED, DECENTRALIZED;

        /**
         * Return the name in lower case.
         */
        @Override
        public String toString() {
            return this.name().toLowerCase();
        }
    }


    /**
     * Enumeration of clock synchronization modes.
     *
     * - OFF: The clock synchronization is universally off.
     * - STARTUP: Clock synchronization occurs at startup only.
     * - ON: Clock synchronization occurs at startup and at runtime.
     *
     * @author{Edward A. Lee <eal@berkeley.edu>}
     */
    public enum ClockSyncMode {
        OFF, INIT, ON; // TODO Discuss initial in now a mode keyword (same as startup) and cannot be used as target property value, thus changed it to init
        // FIXME I could not test if this change breaks anything

        /**
         * Return the name in lower case.
         */
        @Override
        public String toString() {
            return this.name().toLowerCase();
        }
    }

    /**
     * An interface for types associated with target properties.
     *
     * @author{Marten Lohstroh <marten@berkeley.edu>}
     */
    public interface TargetPropertyType {

        /**
         * Return true if the the given Element is a valid instance of this
         * type.
         *
         * @param e The Element to validate.
         * @return True if the element conforms to this type, false otherwise.
         */
        public boolean validate(Element e);

        /**
         * Check (recursively) the given Element against its associated type(s)
         * and add found problems to the given list of errors.
         *
         * @param e    The Element to type check.
         * @param name The name of the target property.
         * @param v    A reference to the validator to report errors to.
         */
        public void check(Element e, String name, LFValidator v);

        /**
         * Helper function to produce an error during type checking.
         *
         * @param name        The description of the target property.
         * @param description The description of the type.
         * @param v           A reference to the validator to report errors to.
         */
        public static void produceError(String name, String description,
                LFValidator v) {
            v.getTargetPropertyErrors().add("Target property '" + name
                    + "' is required to be " + description + ".");
        }
    }

    /**
     * Primitive types for target properties, each with a description used in
     * error messages and predicate used for validating values.
     *
     * @author{Marten Lohstroh <marten@berkeley.edu>}
     */
    public enum PrimitiveType implements TargetPropertyType {
        BOOLEAN("'true' or 'false'",
                v -> ASTUtils.elementToSingleString(v).equalsIgnoreCase("true")
                        || ASTUtils.elementToSingleString(v).equalsIgnoreCase("false")),
        INTEGER("an integer", v -> {
            try {
                Integer.parseInt(ASTUtils.elementToSingleString(v));
            } catch (NumberFormatException e) {
                return false;
            }
            return true;
        }),
        NON_NEGATIVE_INTEGER("a non-negative integer", v -> {
            try {
                int result = Integer.parseInt(ASTUtils.elementToSingleString(v));
                if (result < 0)
                    return false;
            } catch (NumberFormatException e) {
                return false;
            }
            return true;
        }),
        TIME_VALUE("a time value with units", v ->
            v.getKeyvalue() == null && v.getArray() == null
                && v.getLiteral() == null && v.getId() == null
                && (v.getTime() == 0 || v.getUnit() != null)),
        STRING("a string", v -> v.getLiteral() != null && !isCharLiteral(v.getLiteral()) || v.getId() != null),
        FILE("a path to a file", STRING.validator);

        /**
         * A description of this type, featured in error messages.
         */
        private final String description;

        /**
         * A predicate for determining whether a given Element conforms to this
         * type.
         */
        public final Predicate<Element> validator;

        /**
         * Private constructor to create a new primitive type.
         * @param description A textual description of the type that should
         * start with "a/an".
         * @param validator A predicate that returns true if a given Element
         * conforms to this type.
         */
        private PrimitiveType(String description,
                Predicate<Element> validator) {
            this.description = description;
            this.validator = validator;
        }

        /**
         * Return true if the the given Element is a valid instance of this type.
         */
        public boolean validate(Element e) {
            return this.validator.test(e);
        }

        /**
         * Check (recursively) the given Element against its associated type(s)
         * and add found problems to the given list of errors.
         *
         * @param e      The element to type check.
         * @param name   The name of the target property.
         * @param v      The validator to which any errors should be reported.
         */
        public void check(Element e, String name, LFValidator v) {
            if (!this.validate(e)) {
                TargetPropertyType.produceError(name, this.description, v);
            }
            // If this is a file, perform an additional check to make sure
            // the file actually exists.
            // FIXME: premature because we first need a mechanism for looking up files!
            // Looking in the same directory is too restrictive. Disabling this check for now.
            /*
            if (this == FILE) {
                String file = ASTUtils.toSingleString(e);

                if (!FileConfig.fileExists(file, FileConfig.toPath(e.eResource().getURI()).toFile().getParent())) {
                    v.targetPropertyWarnings
                            .add("Could not find file: '" + file + "'.");
                }
            }
            */
        }

        /**
         * Return a textual description of this type.
         */
        @Override
        public String toString() {
            return this.description;
        }


        private static boolean isCharLiteral(String s) {
            return s.length() > 2
                && '\'' == s.charAt(0)
                && '\'' == s.charAt(s.length() - 1);
        }
    }

    /**
     * Clock synchronization options.
     * @author{Marten Lohstroh <marten@berkeley.edu>}
     */
    public enum ClockSyncOption implements DictionaryElement {
        ATTENUATION("attenuation", PrimitiveType.NON_NEGATIVE_INTEGER),
        LOCAL_FEDERATES_ON("local-federates-on", PrimitiveType.BOOLEAN),
        PERIOD("period", PrimitiveType.TIME_VALUE),
        TEST_OFFSET("test-offset", PrimitiveType.TIME_VALUE),
        TRIALS("trials", PrimitiveType.NON_NEGATIVE_INTEGER),
        COLLECT_STATS("collect-stats", PrimitiveType.BOOLEAN);

        public final PrimitiveType type;

        private final String description;

        private ClockSyncOption(String alias, PrimitiveType type) {
            this.description = alias;
            this.type = type;
        }


        /**
         * Return the description of this dictionary element.
         */
        @Override
        public String toString() {
            return this.description;
        }

        /**
         * Return the type associated with this dictionary element.
         */
        public TargetPropertyType getType() {
            return this.type;
        }
    }

    /**
     * Docker options.
     * @author{Edward A. Lee <eal@berkeley.edu>}
     */
    public enum DockerOption implements DictionaryElement {
        FROM("FROM", PrimitiveType.STRING);

        public final PrimitiveType type;

        private final String description;

        private DockerOption(String alias, PrimitiveType type) {
            this.description = alias;
            this.type = type;
        }

        /**
         * Return the description of this dictionary element.
         */
        @Override
        public String toString() {
            return this.description;
        }

        /**
         * Return the type associated with this dictionary element.
         */
        public TargetPropertyType getType() {
            return this.type;
        }
    }


    /**
     * Platform options.
     * @author{Anirudh Rengarajan <arengarajan@berkeley.edu>}
     */
    public enum PlatformOption implements DictionaryElement {
        NAME("name", PrimitiveType.STRING),
        BAUDRATE("baud-rate", PrimitiveType.NON_NEGATIVE_INTEGER),
        BOARD("board", PrimitiveType.STRING);
        
        public final PrimitiveType type;
        
        private final String description;
        
        private PlatformOption(String alias, PrimitiveType type) {
            this.description = alias;
            this.type = type;
        }
        
        
        /**
         * Return the description of this dictionary element.
         */
        @Override
        public String toString() {
            return this.description;
        }
    
        /**
         * Return the type associated with this dictionary element.
         */
        public TargetPropertyType getType() {
            return this.type;
        }
    }

    /**
     * Coordination options.
     * @author{Edward A. Lee <eal@berkeley.edu>}
     */
    public enum CoordinationOption implements DictionaryElement {
        ADVANCE_MESSAGE_INTERVAL("advance-message-interval", PrimitiveType.TIME_VALUE);

        public final PrimitiveType type;

        private final String description;

        private CoordinationOption(String alias, PrimitiveType type) {
            this.description = alias;
            this.type = type;
        }


        /**
         * Return the description of this dictionary element.
         */
        @Override
        public String toString() {
            return this.description;
        }

        /**
         * Return the type associated with this dictionary element.
         */
        public TargetPropertyType getType() {
            return this.type;
        }
    }

    /**
     * Log levels in descending order of severity.
     * @author{Marten Lohstroh <marten@berkeley.edu>}
     */
    public enum LogLevel {
        ERROR, WARN, INFO, LOG, DEBUG;

        /**
         * Return the name in lower case.
         */
        @Override
        public String toString() {
            return this.name().toLowerCase();
        }
    }

    /**
     * Enumeration of supported platforms
     */
    public enum Platform {
        AUTO,
        ARDUINO("Arduino"),
        LINUX("Linux"),
        MAC("Darwin"),
        WINDOWS("Windows");

        String cMakeName;
        Platform() {
            this.cMakeName = this.toString();
        }
        Platform(String cMakeName) {
            this.cMakeName = cMakeName;
        }

        /**
         * Return the name in lower case.
         */
        @Override
        public String toString() {
            return this.name().toLowerCase();
        }

        /**
         * Get the CMake name for the platform.
         */
        public String getcMakeName() {
            return this.cMakeName;
        }
    }

    /**
     * Supported schedulers.
     * @author{Soroush Bateni <soroush@utdallas.edu>}
     */
    public enum SchedulerOption {
        NP(false),         // Non-preemptive
        adaptive(false, List.of(
            Path.of("scheduler_adaptive.c"),
            Path.of("worker_assignments.h"),
            Path.of("worker_states.h"),
            Path.of("data_collection.h")
        )),
        GEDF_NP(true),    // Global EDF non-preemptive
        GEDF_NP_CI(true); // Global EDF non-preemptive with chain ID
        // PEDF_NP(true);    // Partitioned EDF non-preemptive (FIXME: To be re-added in a future PR)

        /**
         * Indicate whether or not the scheduler prioritizes reactions by deadline.
         */
        private final boolean prioritizesDeadline;

        /** Relative paths to files required by this scheduler. */
        private final List<Path> relativePaths;

        SchedulerOption(boolean prioritizesDeadline) {
            this(prioritizesDeadline, null);
        }

        SchedulerOption(boolean prioritizesDeadline, List<Path> relativePaths) {
            this.prioritizesDeadline = prioritizesDeadline;
            this.relativePaths = relativePaths;
        }

        /**
         * Return true if the scheduler prioritizes reactions by deadline.
         */
        public boolean prioritizesDeadline() {
            return this.prioritizesDeadline;
        }

        public List<Path> getRelativePaths() {
            return relativePaths != null ? ImmutableList.copyOf(relativePaths) :
                   List.of(Path.of("scheduler_" + this + ".c"));
        }

        public static SchedulerOption getDefault() {
            return NP;
        }
    }

    /**
     * Tracing options.
     * @author{Edward A. Lee <eal@berkeley.edu>}
     */
    public enum TracingOption implements DictionaryElement {
        TRACE_FILE_NAME("trace-file-name", PrimitiveType.STRING);

        public final PrimitiveType type;

        private final String description;

        private TracingOption(String alias, PrimitiveType type) {
            this.description = alias;
            this.type = type;
        }

        /**
         * Return the description of this dictionary element.
         */
        @Override
        public String toString() {
            return this.description;
        }

        /**
         * Return the type associated with this dictionary element.
         */
        public TargetPropertyType getType() {
            return this.type;
        }
    }

}
