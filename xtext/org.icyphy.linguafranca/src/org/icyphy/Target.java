/* Static information about targets. */
/** 
 * Copyright (c) 2019, The University of California at Berkeley.
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
 * THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.icyphy;

import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Predicate;
import java.util.stream.Collectors;

import org.icyphy.linguaFranca.Array;
import org.icyphy.linguaFranca.Element;
import org.icyphy.linguaFranca.KeyValuePair;
import org.icyphy.linguaFranca.KeyValuePairs;
import org.icyphy.linguaFranca.TimeUnit;

/** 
 * Enumeration of targets and their associated properties. These classes are
 * written in Java, not Xtend, because the enum implementation in Xtend more
 * primitive. It is safer to use enums rather than string values since it allows
 * faulty references to be caught at compile time. Switch statements that take
 * as input an enum but do not have cases for all members of the enum are also
 * reported by Xtend with a warning message.
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
public enum Target {
    C("C", true, Arrays.asList(
                // List via: https://en.cppreference.com/w/c/keyword
                "auto",
                "break",
                "case",
                "char",
                "const",
                "continue",
                "default",
                "do",
                "double",
                "else",
                "enum",
                "extern",
                "float",
                "for",
                "goto",
                "if",
                "inline", // (since C99)
                "int",
                "long",
                "register",
                "restrict", // (since C99)
                "return",
                "short",
                "signed",
                "sizeof",
                "static",
                "struct",
                "switch",
                "typedef",
                "union",
                "unsigned",
                "void",
                "volatile",
                "while",
                "_Alignas", // (since C11)
                "_Alignof", // (since C11)
                "_Atomic", // (since C11)
                "_Bool", // (since C99)
                "_Complex", // (since C99)
                "_Generic", // (since C11)
                "_Imaginary", // (since C99)
                "_Noreturn", // (since C11)
                "_Static_assert", // (since C11)
                "_Thread_local" // (since C11)
                )
    ), CCPP("CCpp", true, Target.C.keywords), 
    CPP("Cpp", true, Arrays.asList(
                // List via: https://en.cppreference.com/w/cpp/keyword
                "alignas", // (since C++11)
                "alignof", // (since C++11)
                "and",
                "and_eq",
                "asm",
                "atomic_cancel", // (TM TS)
                "atomic_commit", // (TM TS)
                "atomic_noexcept", // (TM TS)
                "auto(1)",
                "bitand",
                "bitor",
                "bool",
                "break",
                "case",
                "catch",
                "char",
                "char8_t", // (since C++20)
                "char16_t", // (since C++11)
                "char32_t", // (since C++11)
                "class(1)",
                "compl",
                "concept", // (since C++20)
                "const",
                "consteval", // (since C++20)
                "constexpr", // (since C++11)
                "constinit", // (since C++20)
                "const_cast",
                "continue",
                "co_await", // (since C++20)
                "co_return", // (since C++20)
                "co_yield", // (since C++20)
                "decltype", // (since C++11)
                "default(1)",
                "delete(1)",
                "do",
                "double",
                "dynamic_cast",
                "else",
                "enum",
                "explicit",
                "export(1)(3)",
                "extern(1)",
                "false",
                "float",
                "for",
                "friend",
                "goto",
                "if",
                "inline(1)",
                "int",
                "long",
                "mutable(1)",
                "namespace",
                "new",
                "noexcept", // (since C++11)
                "not",
                "not_eq",
                "nullptr", // (since C++11)
                "operator",
                "or",
                "or_eq",
                "private",
                "protected",
                "public",
                "reflexpr", // (reflection TS)
                "register(2)",
                "reinterpret_cast",
                "requires", // (since C++20)
                "return",
                "short",
                "signed",
                "sizeof(1)",
                "static",
                "static_assert", // (since C++11)
                "static_cast",
                "struct(1)",
                "switch",
                "synchronized", // (TM TS)
                "template",
                "this",
                "thread_local", // (since C++11)
                "throw",
                "true",
                "try",
                "typedef",
                "typeid",
                "typename",
                "union",
                "unsigned",
                "using(1)",
                "virtual",
                "void",
                "volatile",
                "wchar_t",
                "while",
                "xor",
                "xor_eq"
                )
    ),
    TS("TypeScript", false, Arrays.asList(
                // List via: https://github.com/Microsoft/TypeScript/issues/2536 
                // Reserved words
                "break",
                "case",
                "catch",
                "class",
                "const",
                "continue",
                "debugger",
                "default",
                "delete",
                "do",
                "else",
                "enum",
                "export",
                "extends",
                "false",
                "finally",
                "for",
                "function",
                "if",
                "import",
                "in",
                "instanceof",
                "new",
                "null",
                "return",
                "super",
                "switch",
                "this",
                "throw",
                "true",
                "try",
                "typeof",
                "var",
                "void",
                "while",
                "with",
                
                //Strict Mode Reserved Words
                "as",
                "implements",
                "interface",
                "let",
                "package",
                "private",
                "protected",
                "public",
                "static",
                "yield",
                
                // Contextual Keywords
                "any",
                "boolean",
                "constructor",
                "declare",
                "get",
                "module",
                "require",
                "number",
                "set",
                "string",
                "symbol",
                "type",
                "from",
                "of"
                )
    ), 
    Python("Python", false, Arrays.asList(
            // List via: https://www.w3schools.com/python/python_ref_keywords.asp
            // and https://en.cppreference.com/w/c/keyword (due to reliance on the C lib).
            "and",
            "as",
            "assert",
            "auto",
            "break",
            "case",
            "char",
            "class",
            "const",
            "continue",
            "def",
            "default",
            "del",
            "do",
            "double",
            "elif",
            "else",
            "enum",
            "except",
            "extern",
            "False",
            "finally",
            "float",
            "for",
            "from",
            "global",
            "goto",
            "if",
            "import",
            "inline", // (since C99)
            "int",
            "in",
            "is",
            "lambda",
            "long",
            "None",
            "nonlocal",
            "not",
            "or",
            "pass",
            "raise",
            "register",
            "restrict", // (since C99)
            "return",
            "short",
            "signed",
            "sizeof",
            "static",
            "struct",
            "switch",
            "True",
            "try",
            "typedef",
            "union",
            "unsigned",
            "void",
            "volatile",
            "while",
            "with",
            "yield",
            "_Alignas", // (since C11)
            "_Alignof", // (since C11)
            "_Atomic", // (since C11)
            "_Bool", // (since C99)
            "_Complex", // (since C99)
            "_Generic", // (since C11)
            "_Imaginary", // (since C99)
            "_Noreturn", // (since C11)
            "_Static_assert", // (since C11)
            "_Thread_local" // (since C11)
            )
    );
    
    /**
     * String representation of this target.
     */
    private final String description;
        
    /**
     * Whether or not this target requires types.
     */
    public final boolean requiresTypes;
    
    /**
     * Reserved words in the target language.
     */
    public final List<String> keywords;
    
    /**
     * Return an array of all known targets.
     */
    public final static Target[] ALL = Target.values();
    
    /**
     * Private constructor for targets.
     * @param name String representation of this target.
     * @param requires Types Whether this target requires type annotations or not.
     * @param keywords List of reserved strings in the target language.
     */
    private Target(String alias, boolean requiresTypes, List<String> keywords) {
        this.description = alias;
        this.requiresTypes = requiresTypes;
        this.keywords = keywords;
    }
    
    /**
     * Check whether a given string corresponds with the name of a valid target.
     * @param string The string to find a matching target for.
     * @return true if a matching target was found, false otherwise.
     */
    public final static boolean matches(String string) {
        if (Target.match(string) != null) {
            return true;
        }
        return false;
    }

    /**
     * Return the entry that matches the given string.
     * @param string The string to match against.
     */
    public static Target match(String string) {
        return (Target)Target.doMatch(string, Target.values());
    }

    /**
     * Return the alias.
     */
    @Override
    public String toString() {
        return this.description;
    }
    
    // Inner classes.
    
    /**
     * All target properties along with a list of targets that supports them.
     * @author{Marten Lohstroh <marten@berkeley.edu>}
     */
    public enum TargetProperties {
        
        /**
         * Directive to let the generator use the custom build command.
         */
        BUILD("build", UnionType.STRING_OR_STRING_ARRAY, Arrays.asList(Target.C), (config, value) -> {
            config.buildCommands = ASTUtils.toListOfStrings(value);
        }),
        
        /**
         * Directive to specify the target build type such as 'Release' or 'Debug'.
         */
        BUILD_TYPE("build-type", UnionType.BUILD_TYPE_UNION, Arrays.asList(Target.CPP), (config, value) -> {
            config.cmakeBuildType = (BuildType) UnionType.BUILD_TYPE_UNION.match(ASTUtils.toText(value));
        }),
        
        /**
         * Directive to let the federate execution handle clock synchronization in software.
         */
        CLOCK_SYNC("clock-sync", UnionType.CLOCK_SYNC_UNION, Arrays.asList(Target.C), (config, value) -> {
            config.clockSync = (ClockSyncMode) UnionType.CLOCK_SYNC_UNION.match(ASTUtils.toText(value));
        }),

        /**
         * Key-value pairs giving options for clock synchronization.
         */
        CLOCK_SYNC_OPTIONS("clock-sync-options", DictionaryType.CLOCK_SYNC_OPTION_DICT, Arrays.asList(Target.C), (config, value) -> {
            for (KeyValuePair entry: value.getKeyvalue().getPairs()) {
                // FIXME: convert to string values here, not later on during code generation.
                config.clockSyncOptions.put((ClockSyncOption) DictionaryType.CLOCK_SYNC_OPTION_DICT.match(entry.getName()), entry.getValue());
            }
        }),

        /**
         * Directive to specify a cmake to be included by the generated build systems.
         *
         * This gives full control over the C++ build as any cmake parameters can be adjusted in the included file.
         */
        CMAKE_INCLUDE("cmake-include", PrimitiveType.STRING, Arrays.asList(Target.CPP), (config, value) -> {
            config.cmakeInclude = ASTUtils.toText(value);
        }),
        
        /**
         * Directive to specify the target compiler.
         */
        COMPILER("compiler", PrimitiveType.STRING,  Arrays.asList(Target.ALL), (config, value) -> {
            config.compiler = ASTUtils.toText(value);
        }),
        
        /**
         * Directive to let the execution engine allow logical time to elapse
         * faster than physical time.
         */
        FAST("fast", PrimitiveType.BOOLEAN, Arrays.asList(Target.ALL), (config, value) -> {
            config.fastMode = ASTUtils.toBoolean(value);
        }),
        
        /**
         * Directive to stage particular files on the class path to be
         * processed by the code generator.
         */
        FILES("files", UnionType.STRING_OR_STRING_ARRAY, Arrays.asList(Target.ALL), (config, value) -> {
            config.fileNames = ASTUtils.toListOfStrings(value);
        }),
        
        /**
         * Flags to be passed on to the target compiler.
         */
        FLAGS("flags", PrimitiveType.STRING, Arrays.asList(Target.C, Target.CCPP), (config, value) -> {
            config.compilerFlags.clear();
            String str = ASTUtils.toText(value);
            if (!str.isEmpty()) {
                // FIXME: this is unsafe because it doesn't account for whitespace within quotes.
                Arrays.asList(str.split("\\s"))
                        .forEach(sw -> config.compilerFlags.add(sw));
            }
        }),
        
        /**
         * Directive to specify the coordination mode
         */
        COORDINATION("coordination", UnionType.COORDINATION_UNION, Arrays.asList(Target.C, Target.CCPP, Target.Python), (config, value) -> {
            config.coordination = (CoordinationType) UnionType.COORDINATION_UNION.match(ASTUtils.toText(value));
        }),
        
        /**
         * Directive to let the execution engine remain active also if there
         * are no more events in the event queue.
         */
        KEEPALIVE("keepalive", PrimitiveType.BOOLEAN, Arrays.asList(Target.ALL), (config, value) -> {
            config.keepalive = ASTUtils.toBoolean(value);
        }),
        
        /**
         * Directive to specify the grain at which to report log messages during execution.
         */
        LOGGING("logging", UnionType.LOGGING_UNION, Arrays.asList(Target.ALL), (config, value) -> {
            config.logLevel = (LogLevel) UnionType.LOGGING_UNION.match(ASTUtils.toText(value));
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
        PROTOBUFS("protobufs", UnionType.STRING_OR_STRING_ARRAY, Arrays.asList(Target.C, Target.TS, Target.Python), (config, value) -> {
            config.protoFiles = ASTUtils.toListOfStrings(value);
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
         * Directive to let the runtime produce execution traces.
         */
        TRACING("tracing", PrimitiveType.BOOLEAN,
                Arrays.asList(Target.C, Target.CPP), (config, value) -> {
                    config.tracing = ASTUtils.toBoolean(value);
                });
        /**
         * String representation of this target property.
         */
        public final String alias;
        
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
        
        public final BiConsumer<Configuration, Element> setter;
        
        /**
         * Private constructor for target properties.
         * @param name String representation of this property.
         * @param supportedBy List of targets that support this property.
         */
        private TargetProperties(String alias, TargetPropertyType type, List<Target> supportedBy, BiConsumer<Configuration, Element> setter) {
            this.alias = alias;
            this.type = type;
            this.supportedBy = supportedBy;
            this.setter = setter;
        }

        /**
         * Update the given configuration using the given target properties.
         * @param config The configuration object to update.
         * @param properties Target properties that inform the configuration.
         */
        public static void update(Configuration config, List<KeyValuePair> properties) {
            properties.forEach(property -> match(property.getName()).setter
                    .accept(config, property.getValue()));
        }
        
        /**
         * Return the entry that matches the given string.
         * @param string The string to match against.
         */
        public static TargetProperties match(String string) {
            return (TargetProperties)Target.doMatch(string, TargetProperties.values());
        }

        public static List<TargetProperties> getOptions() {
            return Arrays.asList(TargetProperties.values());
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
     * Interface for dictionary elements. It associates an entry with a type.
     */
    public interface DictionaryElement {
        public TargetPropertyType getType();
    }
    
    public enum DictionaryType implements TargetPropertyType {
        CLOCK_SYNC_OPTION_DICT(Arrays.asList(ClockSyncOption.values()));
        
        /**
         * The key-value pairs that are allowed in this dictionary.
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
        
        public DictionaryElement match(String string) {
            return (DictionaryElement) Target.doMatch(string, options.toArray());
        }
        
        /**
         * Recursively check that the passed in element conforms to the rules of
         * this dictionary.
         */
        @Override
        public void check(Element e, String name, List<String> errors) {
            KeyValuePairs kv = e.getKeyvalue();
            if (kv == null) {
                TargetPropertyType.produceError(name, this.toString(),
                        errors);
            } else {
                for (KeyValuePair pair : kv.getPairs()) {
                    String key = pair.getName();
                    Element val = pair.getValue();
                    Optional<DictionaryElement> match = this.options.stream()
                            .filter(element -> key.equalsIgnoreCase(element.toString())).findAny();
                    if (match.isPresent()) {
                        // Make sure the type is correct, too.
                        TargetPropertyType type = match.get().getType();
                        type.check(val, name + "." + key, errors);
                    } else {
                        // No match found; report error.
                        TargetPropertyType.produceError(name,
                                this.toString(), errors);
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
        
    public enum UnionType implements TargetPropertyType {
        STRING_OR_STRING_ARRAY(
                Arrays.asList(PrimitiveType.STRING, ArrayType.STRING_ARRAY), null),
        BUILD_TYPE_UNION(Arrays.asList(BuildType.values()), null),
        COORDINATION_UNION(Arrays.asList(CoordinationType.values()), CoordinationType.CENTRALIZED),
        LOGGING_UNION(Arrays.asList(LogLevel.values()), LogLevel.INFO),
        CLOCK_SYNC_UNION(Arrays.asList(ClockSyncMode.values()), ClockSyncMode.INITIAL);

        public final List<Enum<?>> options;
        
        private final Enum<?> defaultOption;
        
        private UnionType(List<Enum<?>> list, Enum<?> defaultOption) {
            this.options = list;
            this.defaultOption = defaultOption; 
        }
        
        public Enum<?> match(String string) {
            return (Enum<?>) Target.doMatch(string, options.toArray());
        }
        
        @Override
        public void check(Element e, String name, List<String> errors) {
            Optional<Enum<?>> match = this.match(e);
            if (match.isPresent()) {
                // Go deeper if the element is an array or dictionary.
                Enum<?> type = match.get();
                if (type instanceof DictionaryType) {
                    ((DictionaryType) type).check(e, name, errors);
                } else if (type instanceof ArrayType) {
                    ((ArrayType) type).check(e, name, errors);
                }
            } else {
                // No match found; report error.
                TargetPropertyType.produceError(name, this.toString(), errors);
            }
        }
        

        /**
         * 
         * @param e
         * @return
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
         * Return a human-readable description of this type.
         */
        @Override
        public String toString() {
            // FIXME: print (default) after default option.
            return "one of the following: "
                    + options.stream().map(option -> option.toString())
                            .collect(Collectors.joining(", "));
        }

    }
    
    public enum ArrayType implements TargetPropertyType {
        STRING_ARRAY(PrimitiveType.STRING);
        
        public TargetPropertyType type;
        
        private ArrayType(TargetPropertyType type) {
            this.type = type;
        }
        
        @Override
        public void check(Element e, String name, List<String> errors) {
            Array array = e.getArray();
            if (array == null) {
                TargetPropertyType.produceError(name, this.toString(), errors);
            } else {
                List<Element> elements = array.getElements();
                for (int i = 0; i < elements.size(); i++) {
                    this.type.check(elements.get(i), name + "[" + i + "]", errors);
                }
            }
        }
        
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
     * Cmake build types.
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
     * Given a string and a list of candidate objects, return the first candidate
     * that matches, or null if no candidate matches.
     * @param string The string to match against candidates.
     * @param candidates The candidates to match the string against. 
     */
    private static Object doMatch(final String string, final Object[] candidates) {
        return Arrays.stream(candidates)
                .filter(e -> (e.toString().equalsIgnoreCase(string)))
                .findAny().orElse(null);
    }
    
    /**
     * Enumeration of clock synchronization modes.
     * OFF: The clock synchronization is universally off.
     * STARTUP: Clock synchronization occurs at startup only.
     * ON: Clock synchronization occurs at startup and at runtime.
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
     * @author Marten Lohstroh
     */
    public interface TargetPropertyType {
        
        /**
         * Return true if the the given Element is a valid instance of this type.
         * @param e The Element to validate.
         */
        public boolean validate(Element e);
        
        /**
         * Check (recursively) the given Element against its associated type(s)
         * and add found problems to the given list of errors.
         * @param e The Element to type check.
         * @param name The name of the target property.
         * @param errors A list of errors to append to if problems are found.
         */
        public void check(Element e, String name, List<String> errors);
        
        /**
         * Helper function to produce an error during type checking.
         * @param name The description of the target property.
         * @param description The description of the type.
         * @param errors The list of errors to append to.
         */
        public static void produceError(String name, String description, List<String> errors) {
            errors.add("Target property '" + name + "' is required to be " + description + ".");
        }
    }
    
    /**
     * Primitive types for target properties, each with a description used in
     * error messages and predicate used for validating values.
     * 
     * @author Marten Lohstroh
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
        TIME_VALUE("a time value (with units)", v -> {
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
         // FIXME: Ideally, we'd check whether the file exists, but for that we need to know the current directory.
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
         * @param e The element to type check.
         * @param name The name of the target property.
         * @param errors A list of errors to append to if problems are found.
         */
        public void check(Element e, String name, List<String> errors) {
            if (!this.validate(e)) {
                TargetPropertyType.produceError(name, this.description, errors);
            }
        }

        /**
         * Return a textual description of this type.
         */
        @Override
        public String toString() {
            return this.description;
        }
    }
    
    
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

}
