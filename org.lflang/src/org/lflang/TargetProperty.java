package org.lflang;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import org.lflang.lf.Array;
import org.lflang.lf.Element;
import org.lflang.lf.KeyValuePair;
import org.lflang.lf.KeyValuePairs;
import org.lflang.lf.TimeUnit;
import org.lflang.validation.LFValidator;

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
            Arrays.asList(Target.C), (config, value) -> {
                config.buildCommands = ASTUtils.toListOfStrings(value);
            }),
    
    /**
     * Directive to specify the target build type such as 'Release' or 'Debug'.
     */
    BUILD_TYPE("build-type", UnionType.BUILD_TYPE_UNION,
            Arrays.asList(Target.CPP), (config, value) -> {
                config.cmakeBuildType = (BuildType) UnionType.BUILD_TYPE_UNION
                        .forName(ASTUtils.toText(value));
            }),
    
    /**
     * Directive to let the federate execution handle clock synchronization in software.
     */
    CLOCK_SYNC("clock-sync", UnionType.CLOCK_SYNC_UNION,
            Arrays.asList(Target.C), (config, value) -> {
                config.clockSync = (ClockSyncMode) UnionType.CLOCK_SYNC_UNION
                        .forName(ASTUtils.toText(value));
            }),
    
    /**
     * Key-value pairs giving options for clock synchronization.
     */
    CLOCK_SYNC_OPTIONS("clock-sync-options",
            DictionaryType.CLOCK_SYNC_OPTION_DICT, Arrays.asList(Target.C),
            (config, value) -> {
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
     * This gives full control over the C++ build as any cmake parameters
     * can be adjusted in the included file.
     */
    CMAKE_INCLUDE("cmake-include", PrimitiveType.STRING,
            Arrays.asList(Target.CPP), (config, value) -> {
                config.cmakeInclude = ASTUtils.toText(value);
            }),
    
    /**
     * Directive to specify the target compiler.
     */
    COMPILER("compiler", PrimitiveType.STRING, Arrays.asList(Target.ALL),
            (config, value) -> {
                config.compiler = ASTUtils.toText(value);
            }),
    
    /**
     * Directive to generate a Dockerfile. This is either a boolean,
     * true or false, or a dictionary of options.
     */
    DOCKER("docker", UnionType.DOCKER_UNION,
            Arrays.asList(Target.C), (config, value) -> {
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
                                config.dockerOptions.from = ASTUtils.toText(entry.getValue());
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
            Arrays.asList(Target.CPP), (config, value) -> {
                config.externalRuntimePath = ASTUtils.toText(value);
            }),

    /**
     * Directive to let the execution engine allow logical time to elapse
     * faster than physical time.
     */
    FAST("fast", PrimitiveType.BOOLEAN, Arrays.asList(Target.ALL),
            (config, value) -> {
                config.fastMode = ASTUtils.toBoolean(value);
            }),
    
    /**
     * Directive to stage particular files on the class path to be
     * processed by the code generator.
     */
    FILES("files", UnionType.FILE_OR_FILE_ARRAY, Arrays.asList(Target.ALL),
            (config, value) -> {
                config.fileNames = ASTUtils.toListOfStrings(value);
            }),
    
    /**
     * Flags to be passed on to the target compiler.
     */
    FLAGS("flags", UnionType.STRING_OR_STRING_ARRAY,
            Arrays.asList(Target.C, Target.CCPP), (config, value) -> {
                config.compilerFlags = ASTUtils.toListOfStrings(value);
            }),
    
    /**
     * Directive to specify the coordination mode
     */
    COORDINATION("coordination", UnionType.COORDINATION_UNION,
            Arrays.asList(Target.C, Target.CCPP, Target.Python),
            (config, value) -> {
                config.coordination = (CoordinationType) UnionType.COORDINATION_UNION
                        .forName(ASTUtils.toText(value));
            }),
    
    /**
     * Key-value pairs giving options for clock synchronization.
     */
    COORDINATION_OPTIONS("coordination-options",
            DictionaryType.COORDINATION_OPTION_DICT, Arrays.asList(Target.C),
            (config, value) -> {
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
    KEEPALIVE("keepalive", PrimitiveType.BOOLEAN, Arrays.asList(Target.ALL),
            (config, value) -> {
                config.keepalive = ASTUtils.toBoolean(value);
            }),
    
    /**
     * Directive to specify the grain at which to report log messages during execution.
     */
    LOGGING("logging", UnionType.LOGGING_UNION, Arrays.asList(Target.ALL),
            (config, value) -> {
                config.logLevel = (LogLevel) UnionType.LOGGING_UNION
                        .forName(ASTUtils.toText(value));
            }),
    
    /**
     * Directive to not invoke the target compiler.
     */
    NO_COMPILE("no-compile", PrimitiveType.BOOLEAN,
            Arrays.asList(Target.C, Target.CPP, Target.CCPP),
            (config, value) -> {
                config.noCompile = ASTUtils.toBoolean(value);
            }),
    
    /**
     * Directive to disable validation of reactor rules at runtime.
     */
    NO_RUNTIME_VALIDATION("no-runtime-validation", PrimitiveType.BOOLEAN,
            Arrays.asList(Target.CPP), (config, value) -> {
                config.noRuntimeValidation = ASTUtils.toBoolean(value);
            }),
    
    /**
     * Directive for specifying .proto files that need to be compiled and their
     * code included in the sources.
     */
    PROTOBUFS("protobufs", UnionType.FILE_OR_FILE_ARRAY,
            Arrays.asList(Target.C, Target.TS, Target.Python),
            (config, value) -> {
                config.protoFiles = ASTUtils.toListOfStrings(value);
            }),

    /**
     * Directive for specifying a specific version of the reactor runtime library.
     */
    RUNTIME_VERSION("runtime-version", PrimitiveType.STRING,
            Arrays.asList(Target.CPP), (config, value) -> {
                config.runtimeVersion = ASTUtils.toText(value);
            }),

    /**
     * Directive to specify the number of threads.
     */
    THREADS("threads", PrimitiveType.NON_NEGATIVE_INTEGER,
            Arrays.asList(Target.C, Target.CPP, Target.CCPP),
            (config, value) -> {
                config.threads = ASTUtils.toInteger(value);
            }),
    
    /**
     * Directive to specify the execution timeout.
     */
    TIMEOUT("timeout", PrimitiveType.TIME_VALUE, Arrays.asList(Target.ALL),
            (config, value) -> {
                config.timeout = ASTUtils.toTimeValue(value);
            }),
    
    /**
     * Directive to generate a Dockerfile. This is either a boolean,
     * true or false, or a dictionary of options.
     */
    TRACING("tracing", UnionType.TRACING_UNION,
            Arrays.asList(Target.C, Target.CPP), (config, value) -> {
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
                                config.tracing.traceFileName = ASTUtils.toText(entry.getValue());
                                break;
                            default:
                                break;
                        }
                    }
                }
            });
    
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
     * updates the configuration. It is assumed that validation already
     * occurred, so this code should be straightforward.
     */
    public final BiConsumer<TargetConfig, Element> setter;
    
    /**
     * Private constructor for target properties.
     * 
     * @param description String representation of this property.
     * @param type        The type that values assigned to this property
     *                    should conform to.
     * @param supportedBy List of targets that support this property.
     * @param setter      Function for configuration updates.
     */
    private TargetProperty(String description, TargetPropertyType type,
            List<Target> supportedBy,
            BiConsumer<TargetConfig, Element> setter) {
        this.description = description;
        this.type = type;
        this.supportedBy = supportedBy;
        this.setter = setter;
    }

    /**
     * Update the given configuration using the given target properties.
     * 
     * @param config     The configuration object to update.
     * @param properties AST node that holds all the target properties.
     */
    public static void update(TargetConfig config,
            List<KeyValuePair> properties) {
        properties.forEach(property ->  {
            TargetProperty p = forName(property.getName());
            if (p != null) {
                p.setter.accept(config, property.getValue());
            }
        });
    }

    /**
     * Return the entry that matches the given string.
     * @param name The string to match against.
     */
    public static TargetProperty forName(String name) {
        return (TargetProperty)Target.match(name, TargetProperty.values());
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
        public TargetPropertyType getType();
    }

    /**
     * A dictionary type with a predefined set of possible keys and assignable
     * types.
     * 
     * @author{Marten Lohstroh <marten@berkeley.edu>}
     *
     */
    public enum DictionaryType implements TargetPropertyType {
        CLOCK_SYNC_OPTION_DICT(Arrays.asList(ClockSyncOption.values())),
        DOCKER_DICT(Arrays.asList(DockerOption.values())),
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
            return (DictionaryElement) Target.match(name, options.toArray());
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
        FILE_OR_FILE_ARRAY(
                Arrays.asList(PrimitiveType.FILE, ArrayType.FILE_ARRAY), null),
        BUILD_TYPE_UNION(Arrays.asList(BuildType.values()), null),
        COORDINATION_UNION(Arrays.asList(CoordinationType.values()),
                CoordinationType.CENTRALIZED),
        LOGGING_UNION(Arrays.asList(LogLevel.values()), LogLevel.INFO),
        CLOCK_SYNC_UNION(Arrays.asList(ClockSyncMode.values()),
                ClockSyncMode.INITIAL),
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
            return (Enum<?>) Target.match(name, options.toArray());
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
                    return ASTUtils.toText(e)
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
     * Enumeration of Cmake build types.
     * 
     * @author{Christian Menard <christian.menard@tu-dresden.de>}
     */
    public enum BuildType {
        RELEASE("Release"), 
        DEBUG("Debug"), 
        REL_WITH_DEB_INFO("RelWithDebInfo"), 
        MIN_SIZE_REL("MinSizeRel");
        
        /**
         * Alias used in toString method.
         */
        private String alias;
        
        /**
         * Private constructor for Cmake build types.
         */
        private BuildType(String alias) {
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
        OFF, INITIAL, ON;
        
        
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
            v.targetPropertyErrors.add("Target property '" + name
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
                v -> (ASTUtils.toText(v).equalsIgnoreCase("true")
                        || ASTUtils.toText(v).equalsIgnoreCase("false"))),
        INTEGER("an integer", v -> {
            try {
                Integer.decode(ASTUtils.toText(v));
            } catch (NumberFormatException e) {
                return false;
            }
            return true;
        }), 
        NON_NEGATIVE_INTEGER("a non-negative integer", v -> {
            try {
                Integer result = Integer.decode(ASTUtils.toText(v));
                if (result < 0)
                    return false;
            } catch (NumberFormatException e) {
                return false;
            }
            return true;
        }), 
        TIME_VALUE("a time value with units", v -> {
            if ((v.getKeyvalue() != null || v.getArray() != null
                    || v.getLiteral() != null || v.getId() != null)
                    || (v.getTime() != 0 && v.getUnit() == TimeUnit.NONE)) {
                return false;
            } else {
                return true;
            }
        }), 
        STRING("a string", v -> {
            if (v.getLiteral() == null && v.getId() == null) {
                return false;
            }
            return true;
        }), 
        FILE("a path to a file", v -> {
            return STRING.validator.test(v);
        });
    
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
         * @param errors A list of errors to append to if problems are found.
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
                String file = ASTUtils.toText(e);
                 
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
