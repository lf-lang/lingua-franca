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
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.LfFactory;
import org.lflang.lf.Model;
import org.lflang.lf.TargetDecl;
import org.lflang.target.property.type.TargetPropertyType;
import org.lflang.validation.ValidationReporter;

/**
 * A target properties along with a type and a list of supporting targets that supports it, as well
 * as a function for configuration updates.
 *
 * @author Marten Lohstroh
 */
public enum TargetProperty {

    /** Directive to allow including OpenSSL libraries and process HMAC authentication. */
    AUTH(config -> config.auth),
    /** Directive to let the generator use the custom build command. */
    BUILD(config -> config.buildCommands),

    /**
     * Directive to specify the target build type such as 'Release' or 'Debug'. This is also used in
     * the Rust target to select a Cargo profile.
     */
    BUILD_TYPE(config -> config.buildType),

    /** Directive to let the federate execution handle clock synchronization in software. */
    CLOCK_SYNC(config -> config.clockSync),
    /** Key-value pairs giving options for clock synchronization. */
    CLOCK_SYNC_OPTIONS(config -> config.clockSyncOptions),

    /**
     * Directive to specify a cmake to be included by the generated build systems.
     *
     * <p>This gives full control over the C/C++ build as any cmake parameters can be adjusted in
     * the
     * included file.
     */
    CMAKE_INCLUDE(config -> config.cmakeIncludes),
    /** Directive to specify the target compiler. */
    COMPILER(config -> config.compiler),
    /** Directive to specify compile-time definitions. */
    COMPILE_DEFINITIONS(config -> config.compileDefinitions),
    /**
     * Directive to generate a Dockerfile. This is either a boolean, true or false, or a dictionary of
     * options.
     */
    /** Directive to specify the coordination mode */
    COORDINATION(config -> config.coordination),
    /** Key-value pairs giving options for clock synchronization. */
    COORDINATION_OPTIONS(config -> config.coordinationOptions),

    DOCKER(config -> config.dockerOptions),
    /** Directive for specifying a path to an external runtime to be used for the compiled binary. */
    EXTERNAL_RUNTIME_PATH(config -> config.externalRuntimePath),
    /**
     * Directive to let the execution engine allow logical time to elapse faster than physical time.
     */
    FAST(config -> config.fastMode),
    /**
     * Directive to stage particular files on the class path to be processed by the code generator.
     */
    FILES(config -> config.files),

    /** Flags to be passed on to the target compiler. */
    FLAGS(config -> config.compilerFlags),

    /**
     * Directive to let the execution engine remain active also if there are no more events in the
     * event queue.
     */
    KEEPALIVE(config -> config.keepalive),

    /** Directive to specify the grain at which to report log messages during execution. */
    LOGGING(config -> config.logLevel),

    /** Directive to not invoke the target compiler. */
    NO_COMPILE(config -> config.noCompile),

    /** Directive to disable validation of reactor rules at runtime. */
    NO_RUNTIME_VALIDATION(config -> config.noRuntimeValidation),

    /**
     * Directive to specify the platform for cross code generation. This is either a string of the
     * platform or a dictionary of options that includes the string name.
     */
    PLATFORM((TargetConfig config) -> config.platformOptions),

    /** Directive to instruct the runtime to collect and print execution statistics. */
    PRINT_STATISTICS(config -> config.printStatistics),

    /**
     * Directive for specifying .proto files that need to be compiled and their code included in the
     * sources.
     */
    PROTOBUFS(config -> config.protoFiles),

    /** Directive to specify that ROS2 specific code is generated, */
    ROS2(config -> config.ros2),

    /** Directive to specify additional ROS2 packages that this LF program depends on. */
    ROS2_DEPENDENCIES((TargetConfig config) -> config.ros2Dependencies),

    /** Directive for specifying a specific version of the reactor runtime library. */
    RUNTIME_VERSION(config -> config.runtimeVersion),

    /** Directive for specifying a specific runtime scheduler, if supported. */
    SCHEDULER((TargetConfig config) -> config.schedulerType),
    /** Directive to specify that all code is generated in a single file. */
    SINGLE_FILE_PROJECT(config -> config.singleFileProject),

    /** Directive to indicate whether the runtime should use multi-threading. */
    THREADING(config -> config.threading),
    /** Directive to check the generated verification model. */
    VERIFY(config -> config.verify),

    /** Directive to specify the number of worker threads used by the runtime. */
    WORKERS(config -> config.workers),

    /** Directive to specify the execution timeout. */
    TIMEOUT(config -> config.timeout),

    /** Directive to enable tracing. */
    TRACING(config -> config.tracing),

    /**
     * Directive to let the runtime export its internal dependency graph.
     *
     * <p>This is a debugging feature and currently only used for C++ and Rust programs.
     */
    EXPORT_DEPENDENCY_GRAPH(config -> config.exportDependencyGraph),

    /**
     * Directive to let the runtime export the program structure to a yaml file.
     *
     * <p>This is a debugging feature and currently only used for C++ programs.
     */
    EXPORT_TO_YAML(config -> config.exportToYaml),

    /**
     * List of module files to link into the crate as top-level. For instance, a
     * {@code target Rust {
     * rust-modules: [ "foo.rs" ] }} will cause the file to be copied into the generated project,
     * and
     * the generated {@code main.rs} will include it with a {@code mod foo;}. If one of the paths is
     * a
     * directory, it must contain a {@code mod.rs} file, and all its contents are copied.
     */
    RUST_INCLUDE(config -> config.rust.rustTopLevelModules),

    /** Directive for specifying Cargo features of the generated program to enable. */
    CARGO_FEATURES(config -> config.rust.cargoFeatures),

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
    CARGO_DEPENDENCIES(config -> config.rust.cargoDependencies),

    /**
     * Directs the C or Python target to include the associated C file used for setting up federated
     * execution before processing the first tag.
     */
    FED_SETUP(config -> config.fedSetupPreamble);

    public final PropertyGetter get;

    @FunctionalInterface
    private interface PropertyGetter<T> {

        TargetPropertyConfig<T> get(TargetConfig config);
    }

    TargetProperty(PropertyGetter propertyGetter) {
        this.get = propertyGetter;
    }

    public static void overrideAll(TargetConfig config, Properties properties, MessageReporter err) {
        for (Object key : properties.keySet()) {
            TargetProperty p = forName(key.toString());
            if (p != null) {
                try {
                    p.get.get(config).override(properties.get(key));
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
    public static void setAll(TargetConfig config, List<KeyValuePair> properties, MessageReporter err) {
        properties.forEach(
            property -> {
                TargetProperty p = forName(property.getName());
                if (p != null) {
                    try {
                        p.get.get(config).set(property.getValue(), err);
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
        for (TargetProperty p : config.setByUser) { // FIXME: do not use setByUser
            KeyValuePair kv = LfFactory.eINSTANCE.createKeyValuePair();
            kv.setName(p.toString());
            kv.setValue(p.get.get(config).export());
            if (kv.getValue() != null) {
                res.add(kv);
            }
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
                .filter(pair -> pair.getName().equals(property.toString()))
                .toList();
        assert (properties.size() <= 1);
        return properties.size() > 0 ? properties.get(0) : null;
    }

    public static KeyValuePair getKeyValuePair(Model ast, TargetProperty property) {
        return getKeyValuePair(ast.getTarget().getConfig(), property);
    }

    public void validate(KeyValuePairs pairs, Model ast, TargetConfig config, ValidationReporter reporter) {
        this.get.get(config).validate(getKeyValuePair(pairs, this), ast, config, reporter);
    }

    /**
     * Update the given configuration using the given target properties.
     *
     * @param config The configuration object to update.
     * @param properties AST node that holds all the target properties.
     * @param relativePath The path from the main resource to the resource from which the new
     * properties originate.
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
                    // FIXME: figure out different between update and override
                    // p.updater.parseIntoTargetConfig(config, value, err);
                }
            });
    }

    /**
     * Update one of the target properties, given by 'propertyName'. For convenience, a list of
     * target
     * properties (e.g., taken from a file or resource) can be passed without any filtering. This
     * function will do nothing if the list of target properties doesn't include the property given
     * by
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
        // FIXME
//        properties.stream()
//            .filter(p -> {return p.getName().equals(property.toString());})
//            .findFirst()
//            .map(KeyValuePair::getValue)
//            .ifPresent(value -> property.updater.parseIntoTargetConfig(config, value, err));
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
     * Return the name of the property in as it appears in the target declaration.
     * It may be an invalid identifier in other languages (may contains dashes {@code -}).
     */
    @Override
    public String toString() {
        // Work around because this sole property does not follow the naming convention.
        if (this.equals(FED_SETUP)) {
            return "_fed_setup";
        }
        return this.name().toLowerCase().replaceAll("_", "-");
    }

    /** Interface for dictionary elements. It associates an entry with a type. */
    public interface DictionaryElement {

        TargetPropertyType getType();
    }
}
