/* Static information about targets. */

/*************
Copyright (c) 2019, The University of California at Berkeley.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***************/
package org.icyphy;

import java.util.Arrays;
import java.util.List;

/**
 * Enumeration of targets and their associated properties. These classes are
 * written in Java, not Xtend, because the enum implementation in Xtend more
 * primitive. It is saver to use enums rather than string values since it allows
 * faulty references to be caught at compile time. Switch statements that take
 * as input an enum but do not have cases for all members of the enum are also
 * reported by Xtend with a warning message.
 * 
 * @author{Marten Lohstroh <marten@berkeley.edu>}
 */
public enum Targets {
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
    ), 
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
    );
    
    /**
     * String representation of this target.
     */
    public final String name;
        
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
    public final static Targets[] ALL = Targets.values();
    
    /**
     * 
     * @author{Marten Lohstroh <marten@berkeley.edu>}
     */
    public enum TargetProperties {
        /**
         * Directive to specify a cmake to be included by the generated build systems.
         *
         * This gives full control over the C++ build as any cmake parameters can be adjusted in the included file.
         */
        CMAKE_INCLUDE("cmake-include", Arrays.asList(Targets.CPP)),
        /**
         * Directive to specify the target compiler.
         */
        COMPILER("compiler", Arrays.asList(Targets.ALL)),
        /**
         * Directive to let the execution engine run as fast as possible (and
         */
        FAST("fast", Arrays.asList(Targets.ALL)),
        
        /**
         * Flags to be passed on to the target compiler.
         */
        FLAGS("flags", Arrays.asList(Targets.C)),
        
        /**
         * FIXME
         */
        FEDERATES("federates", Arrays.asList(Targets.C)),
        
        /**
         * FIXME
         */
        KEEPALIVE("keepalive", Arrays.asList(Targets.ALL)),
        
        /**
         * Directive to specify the grain at which to report log messages during execution.
         */
        LOGGING("logging", Arrays.asList(Targets.TS)),
        
        /**
         * Directive to not invoke the target compiler.
         */
        NO_COMPILE("no-compile", Arrays.asList(Targets.C)),
        
        /**
         * Directive to specify the number of threads.
         */
        THREADS("threads", Arrays.asList(Targets.C, Targets.CPP)),
        
        /**
         * Directive to specify the execution timeout.
         */
        TIMEOUT("timeout", Arrays.asList(Targets.ALL));
        
        /**
         * List of targets that support this property. If a property is used for
         * a target that does not support it, a warning reported during
         * validation.
         */
        public final List<Targets> supportedBy;
        
        /**
         * String representation of this target property.
         */
        public final String name;
        
        /**
         * Private constructor for target properties.
         * @param name String representation of this property.
         * @param supportedBy List of targets that support this property.
         */
        private TargetProperties(String name, List<Targets> supportedBy) {
            this.name = name;
            this.supportedBy = supportedBy;
        }

        /**
         * Check whether a given string corresponds with the name of a valid target property.
         * @param name The name to find a matching target property for.
         * @return true if a matching property was found, false otherwise.
         */
        public final static boolean isValidName(String name) {
            if (TargetProperties.get(name) != null) {
                return true;
            }
            return false;
        }
        
        /**
         * Return the target property that corresponds with the given string.
         * @param name The name to find a matching target property for.
         * @return a matching target property, null otherwise.
         */
        public final static TargetProperties get(String name) {
            for (TargetProperties p : TargetProperties.values()) {
                if (p.toString().equalsIgnoreCase(name))
                    return p;
            }
            return null;
        }

        /**
         * Print the name of this target property.
         */
        @Override
        public String toString() {
            return this.name;
        }
    }
    
    /**
     * Log levels in descending order of severity.
     * @author{Marten Lohstroh <marten@berkeley.edu>}
     */
    public enum LoggingLevels {
        ERROR, WARN, INFO, LOG, DEBUG;
    }

    /**
     * Private constructor for targets.
     * @param name String representation of this target.
     * @param requires Types Whether this target requires type annotations or not.
     * @param keywords List of reserved strings in the target language.
     */
    private Targets(String name, boolean requiresTypes, List<String> keywords) {
        this.name = name;
        this.requiresTypes = requiresTypes;
        this.keywords = keywords;
    }

    /**
     * Check whether a given string corresponds with the name of a valid target.
     * @param name The name to find a matching target for.
     * @return true if a matching target was found, false otherwise.
     */
    public final static boolean isValidName(String name) {
        if (Targets.get(name) != null) {
            return true;
        }
        return false;
    }

    /**
     * Return the target that corresponds with the given string.
     * @param name The name to find a matching target for.
     * @return a matching target, null otherwise.
     */
    public final static Targets get(String name) {
        for (Targets t : Targets.values()) {
            if (t.toString().equalsIgnoreCase(name))
                return t;
        }
        return null;
    }

    /**
     * Print the name of this target property.
     */
    @Override
    public String toString() {
        return this.name;
    }

}
