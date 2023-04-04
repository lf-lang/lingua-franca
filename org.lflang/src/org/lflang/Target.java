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
package org.lflang;

import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.lflang.lf.TargetDecl;

/** 
 * Enumeration of targets and their associated properties. These classes are
 * written in Java, not Xtend, because the enum implementation in Xtend more
 * primitive. It is safer to use enums rather than string values since it allows
 * faulty references to be caught at compile time. Switch statements that take
 * as input an enum but do not have cases for all members of the enum are also
 * reported by Xtend with a warning message.
 * 
 * @author Marten Lohstroh
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
    CPP("Cpp", true, "cpp", "Cpp", Arrays.asList(
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
    TS("TypeScript", false, "ts", "TS", Arrays.asList(
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
                
                // Strict Mode Reserved Words
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
                "of",
                
                // Reactor-TS specific keywords (other classes, which are less user-facing, have double underscores)
                "TimeUnit",
                "TimeValue",
                "Present",
                "Sched",
                "Read",
                "Write",
                "ReadWrite"
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
    ),
    Rust("Rust", true,
         "rust", "Rust",
         // In our Rust implementation, the only reserved keywords
         // are those that are a valid expression. Others may be escaped
         // with the syntax r#keyword.
         Arrays.asList("self", "true", "false")
    );

    /**
     * String representation of this target.
     */
    private final String displayName;

    /**
     * Name of package containing Kotlin classes for the target language.
     */
    public final String packageName;

    /**
     * Prefix of names of Kotlin classes for the target language.
     */
    public final String classNamePrefix;

    /**
     * Whether or not this target requires types.
     */
    public final boolean requiresTypes;

    /**
     * Reserved words in the target language.
     */
    public final Set<String> keywords;

    /**
     *An unmodifiable list of all known targets.
     */
    public static final List<Target> ALL = List.of(Target.values());

    /**
     * Private constructor for targets.
     *
     * @param displayName     String representation of this target.
     * @param requiresTypes   Types Whether this target requires type annotations or not.
     * @param packageName     Name of package containing Kotlin classes for the target language.
     * @param classNamePrefix Prefix of names of Kotlin classes for the target language.
     * @param keywords        List of reserved strings in the target language.
     */
    Target(String displayName, boolean requiresTypes, String packageName,
           String classNamePrefix, Collection<String> keywords) {
        this.displayName = displayName;
        this.requiresTypes = requiresTypes;
        this.keywords = Collections.unmodifiableSet(new LinkedHashSet<>(keywords));
        this.packageName = packageName;
        this.classNamePrefix = classNamePrefix;
    }


    /**
     * Private constructor for targets without packageName and classNamePrefix.
     */
    Target(String displayName, boolean requiresTypes, Collection<String> keywords) {
        this(displayName, requiresTypes, "N/A", "N/A", keywords); // FIXME: prefix
    }


    /**
     * Return the target whose {@linkplain #getDisplayName() display name}
     * is the given string (modulo character case), or an empty
     * optional if there is no such target.
     */
    public static Optional<Target> forName(String name) {
        return Arrays.stream(Target.values())
                     .filter(it -> it.getDisplayName().equalsIgnoreCase(name))
                     .findFirst();
    }

    /**
     * Return the display name of the target, as it should be
     * written in LF code. This is hence a single identifier.
     * Eg for {@link #CPP} returns {@code "Cpp"}, for {@link #Python}
     * returns {@code "Python"}. Avoid using either {@link #name()}
     * or {@link #toString()}, which have unrelated contracts.
     */
    public String getDisplayName() {
        return displayName;
    }

    /**
     * Returns the conventional directory name for this target.
     * This is used to divide e.g. the {@code test} and {@code example}
     * directories by target language. For instance, {@code test/Cpp}
     * is the path of {@link #CPP}'s test directory, and this
     * method returns {@code "Cpp"}.
     */
    public String getDirectoryName() {
        return displayName;
    }

    /**
     * Return the description. Avoid depending on this, toString
     * is supposed to be debug information. Prefer {@link #getDisplayName()}.
     */
    @Override
    public String toString() {
        return displayName;
    }

    /**
     * Returns whether the given identifier is invalid as the
     * name of an LF construct. This usually means that the identifier
     * is a keyword in the target language. In Rust, many
     * keywords may be escaped with the syntax {@code r#keyword},
     * and they are considered valid identifiers.
     */
    public boolean isReservedIdent(String ident) {
        return this.keywords.contains(ident);
    }

    /**
     * Return true if the target supports multiports and banks
     * of reactors.
     */
    public boolean supportsMultiports() {
        switch (this) {
        case C:
        case CCPP:
        case CPP:
        case Python:
        case Rust:
        case TS:
            return true;
        }
        return false;
    }


    /**
     * Return true if the target supports widths of banks and
     * multiports that depend on reactor parameters (not only
     * on constants).
     */
    public boolean supportsParameterizedWidths() {
        return switch (this) {
            case C, CCPP, CPP, Python, Rust, TS -> true;
        };
    }

    /**
     * Return true if this code for this target should be built using Docker if Docker is used.
     * @return
     */
    public boolean buildsUsingDocker() {
        return switch (this) {
            case TS -> false;
            case C, CCPP, CPP, Python, Rust -> true;
        };
    }

    /**
     * Return a string that demarcates the beginning of a single-line comment.
     */
    public String getSingleLineCommentPrefix() {
        return this.equals(Target.Python) ? "#" : "//";
    }

    /**
     * Return true if the keepalive option is set automatically
     * for this target if physical actions are detected in the
     * program (and keepalive was not explicitly unset by the user).
     */
    public boolean setsKeepAliveOptionAutomatically() {
        return this != Rust;
    }

    /**
     * Given a string and a list of candidate objects, return the first
     * candidate that matches, or null if no candidate matches.
     *
     * todo move to CollectionUtil (introduced in #442)
     *
     * @param string     The string to match against candidates.
     * @param candidates The candidates to match the string against.
     */
    public static <T> T match(final String string, final Iterable<T> candidates) {
        // kotlin: candidates.firstOrNull { it.toString().equalsIgnoreCase(string) }
        for (T candidate : candidates) {
            if (candidate.toString().equalsIgnoreCase(string)) {
                return candidate;
            }
        }
        return null;
    }


    /**
     * Given a string and a list of candidate objects, return the first
     * candidate that matches, or null if no candidate matches.
     *
     * todo move to CollectionUtil (introduced in #442)
     *
     * @param string     The string to match against candidates.
     * @param candidates The candidates to match the string against.
     */
    public static <T> T match(final String string, final T[] candidates) {
        return match(string, Arrays.asList(candidates));
    }


    /**
     * Return the target constant corresponding to given target
     * declaration among. Return a non-null result, will throw
     * if invalid.
     *
     * @throws RuntimeException If no {@link TargetDecl} is present or if it is invalid.
     */
    public static Target fromDecl(TargetDecl targetDecl) {
        String name = targetDecl.getName();
        return Target.forName(name)
                     .orElseThrow(() -> new RuntimeException("Invalid target name '" + name + "'"));
    }

}
