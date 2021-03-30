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

/** 
 * Enumeration of targets and their associated properties. These classes are
 * written in Java, not Xtend, because the enum implementation in Xtend more
 * primitive. It is safer to use enums rather than string values since it allows
 * faulty references to be caught at compile time. Switch statements that take
 * as input an enum but do not have cases for all members of the enum are also
 * reported by Xtend with a warning message.
 * 
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
    ), 
    CCPP("CCpp", true, Target.C.keywords), 
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
     * 
     * @param name String representation of this target.
     * @param requires Types Whether this target requires type annotations or not.
     * @param keywords List of reserved strings in the target language.
     */
    private Target(String description, boolean requiresTypes, List<String> keywords) {
        this.description = description;
        this.requiresTypes = requiresTypes;
        this.keywords = keywords;
    }
    
    /**
     * Check whether a given string corresponds with the name of a valid target.
     * @param name The string for which to determine whether there is a match.
     * @return true if a matching target was found, false otherwise.
     */
    public final static boolean hasForName(String name) {
        if (Target.forName(name) != null) {
            return true;
        }
        return false;
    }

    /**
     * Return the target that matches the given string.
     * 
     * @param name The string to match against.
     * @return The matching target (or null if there is none).
     */
    public static Target forName(String name) {
        return (Target)Target.match(name, Target.values());
    }

    /**
     * Return the description.
     */
    @Override
    public String toString() {
        return this.description;
    }
    
    /**
     * Given a string and a list of candidate objects, return the first
     * candidate that matches, or null if no candidate matches.
     * 
     * @param string     The string to match against candidates.
     * @param candidates The candidates to match the string against.
     */
    public static Object match(final String string, final Object[] candidates) {
        return Arrays.stream(candidates)
                .filter(e -> (e.toString().equalsIgnoreCase(string)))
                .findAny().orElse(null);
    }
}
