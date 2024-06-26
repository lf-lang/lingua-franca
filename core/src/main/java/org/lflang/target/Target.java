/* Static information about targets. */
/**
 * Copyright (c) 2019, The University of California at Berkeley. Redistribution and use in source
 * and binary forms, with or without modification, are permitted provided that the following
 * conditions are met: 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer. 2. Redistributions in binary form must
 * reproduce the above copyright notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. THIS SOFTWARE IS PROVIDED BY
 * THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING,
 * BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.lflang.target;

import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import net.jcip.annotations.Immutable;
import org.lflang.lf.TargetDecl;
import org.lflang.target.property.AuthProperty;
import org.lflang.target.property.BuildCommandsProperty;
import org.lflang.target.property.BuildTypeProperty;
import org.lflang.target.property.CargoDependenciesProperty;
import org.lflang.target.property.CargoFeaturesProperty;
import org.lflang.target.property.ClockSyncModeProperty;
import org.lflang.target.property.ClockSyncOptionsProperty;
import org.lflang.target.property.CmakeIncludeProperty;
import org.lflang.target.property.CompileDefinitionsProperty;
import org.lflang.target.property.CompilerProperty;
import org.lflang.target.property.CoordinationOptionsProperty;
import org.lflang.target.property.CoordinationProperty;
import org.lflang.target.property.DockerProperty;
import org.lflang.target.property.ExportDependencyGraphProperty;
import org.lflang.target.property.ExportToYamlProperty;
import org.lflang.target.property.ExternalRuntimePathProperty;
import org.lflang.target.property.FilesProperty;
import org.lflang.target.property.KeepaliveProperty;
import org.lflang.target.property.NoRuntimeValidationProperty;
import org.lflang.target.property.NoSourceMappingProperty;
import org.lflang.target.property.PlatformProperty;
import org.lflang.target.property.PrintStatisticsProperty;
import org.lflang.target.property.ProtobufsProperty;
import org.lflang.target.property.PythonVersionProperty;
import org.lflang.target.property.Ros2DependenciesProperty;
import org.lflang.target.property.Ros2Property;
import org.lflang.target.property.RuntimeVersionProperty;
import org.lflang.target.property.RustIncludeProperty;
import org.lflang.target.property.SchedulerProperty;
import org.lflang.target.property.SingleFileProjectProperty;
import org.lflang.target.property.SingleThreadedProperty;
import org.lflang.target.property.TracePluginProperty;
import org.lflang.target.property.TracingProperty;
import org.lflang.target.property.VerifyProperty;
import org.lflang.target.property.WorkersProperty;

/**
 * Enumeration of targets and their associated properties.
 *
 * @author Marten Lohstroh
 */
@Immutable
public enum Target {
  C(
      "C",
      true,
      Arrays.asList(
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
          )),
  CCPP("CCpp", true, Target.C.keywords),
  CPP(
      "Cpp",
      true,
      Arrays.asList(
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
          "xor_eq")),
  TS(
      "TypeScript",
      false,
      Arrays.asList(
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

          // Reactor-TS specific keywords (other classes, which are less user-facing, have double
          // underscores)
          "TimeUnit",
          "TimeValue",
          "Sched",
          "Read",
          "Write",
          "ReadWrite")),
  Python(
      "Python",
      false,
      Arrays.asList(
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
          )),
  Rust(
      "Rust",
      true,
      // In our Rust implementation, the only reserved keywords
      // are those that are a valid expression. Others may be escaped
      // with the syntax r#keyword.
      Arrays.asList("self", "true", "false"));

  /** String representation of this target. */
  private final String displayName;

  /** Whether or not this target requires types. */
  public final boolean requiresTypes;

  /** Reserved words in the target language. */
  private final Set<String> keywords;

  /** An unmodifiable list of all known targets. */
  public static final List<Target> ALL = List.of(Target.values());

  /**
   * Private constructor for targets.
   *
   * @param displayName String representation of this target.
   * @param requiresTypes Types Whether this target requires type annotations or not.
   * @param keywords List of reserved strings in the target language.
   */
  Target(String displayName, boolean requiresTypes, Collection<String> keywords) {
    this.displayName = displayName;
    this.requiresTypes = requiresTypes;
    this.keywords = Collections.unmodifiableSet(new LinkedHashSet<>(keywords));
  }

  /**
   * Return the target whose {@linkplain #getDisplayName() display name} is the given string (modulo
   * character case), or an empty optional if there is no such target.
   */
  public static Optional<Target> forName(String name) {
    return Arrays.stream(Target.values())
        .filter(it -> it.getDisplayName().equalsIgnoreCase(name))
        .findFirst();
  }

  /**
   * Return the display name of the target, as it should be written in LF code. This is hence a
   * single identifier. Eg for {@link #CPP} returns {@code "Cpp"}, for {@link #Python} returns
   * {@code "Python"}. Avoid using either {@link #name()} or {@link #toString()}, which have
   * unrelated contracts.
   */
  public String getDisplayName() {
    return displayName;
  }

  /**
   * Returns the conventional directory name for this target. This is used to divide e.g. the {@code
   * test} and {@code example} directories by target language. For instance, {@code test/Cpp} is the
   * path of {@link #CPP}'s test directory, and this method returns {@code "Cpp"}.
   */
  public String getDirectoryName() {
    return displayName;
  }

  /**
   * Return the description. Avoid depending on this, toString is supposed to be debug information.
   * Prefer {@link #getDisplayName()}.
   */
  @Override
  public String toString() {
    return displayName;
  }

  /**
   * Returns whether the given identifier is invalid as the name of an LF construct. This usually
   * means that the identifier is a keyword in the target language. In Rust, many keywords may be
   * escaped with the syntax {@code r#keyword}, and they are considered valid identifiers.
   */
  public boolean isReservedIdent(String ident) {
    return this.keywords.contains(ident);
  }

  /** Return true if the target supports federated execution. */
  public boolean supportsFederated() {
    return switch (this) {
      case C, CCPP, Python, TS -> true;
      default -> false;
    };
  }

  /** Return true if the target supports reactor inheritance (extends keyword). */
  public boolean supportsInheritance() {
    return switch (this) {
      case C, CCPP, Python -> true;
      default -> false;
    };
  }

  /** Return true if the target supports multiports and banks of reactors. */
  public boolean supportsMultiports() {
    return switch (this) {
      case C, CCPP, CPP, Python, Rust, TS -> true;
      default -> false;
    };
  }

  /**
   * Return true if the target supports widths of banks and multiports that depend on reactor
   * parameters (not only on constants).
   */
  public boolean supportsParameterizedWidths() {
    return true;
  }

  /**
   * Return true of reaction declarations (i.e., reactions without inlined code) are supported by
   * this target.
   */
  public boolean supportsReactionDeclarations() {
    return this.equals(Target.C) || this.equals(Target.CPP);
  }

  /**
   * Whether the target requires using an equal sign to assign a default value to a parameter, or
   * initialize a state variable. All targets mandate an equal sign when passing arguments to a
   * reactor constructor call, regardless of this method.
   */
  public boolean mandatesEqualsInitializers() {
    return this != CPP;
  }

  /** Allow expressions of the form {@code {a, b, c}}. */
  public boolean allowsBracedListExpressions() {
    return this == C || this == CCPP || this == CPP;
  }

  /** Allow expressions of the form {@code [a, b, c]}. */
  public boolean allowsBracketListExpressions() {
    return this == Python || this == TS || this == Rust;
  }

  /** Allow expressions of the form {@code (a, b, c)}. */
  public boolean allowsParenthesisListExpressions() {
    return this == CPP;
  }

  /** Return a string that demarcates the beginning of a single-line comment. */
  public String getSingleLineCommentPrefix() {
    return this.equals(Target.Python) ? "#" : "//";
  }

  /**
   * Return true if the keepalive option is set automatically for this target if physical actions
   * are detected in the program (and keepalive was not explicitly unset by the user).
   */
  public boolean setsKeepAliveOptionAutomatically() {
    return this != Rust && this != CPP;
  }

  /**
   * Given a string and a list of candidate objects, return the first candidate that matches, or
   * null if no candidate matches.
   *
   * <p>todo move to CollectionUtil (introduced in #442)
   *
   * @param string The string to match against candidates.
   * @param candidates The candidates to match the string against.
   */
  public static <T> T match(
      final String string, final Iterable<T> candidates) { // FIXME: use Optional
    // kotlin: candidates.firstOrNull { it.toString().equalsIgnoreCase(string) }
    for (T candidate : candidates) {
      if (candidate.toString().equalsIgnoreCase(string)) {
        return candidate;
      }
    }
    return null;
  }

  /**
   * Given a string and a list of candidate objects, return the first candidate that matches, or
   * null if no candidate matches.
   *
   * <p>todo move to CollectionUtil (introduced in #442)
   *
   * @param string The string to match against candidates.
   * @param candidates The candidates to match the string against.
   */
  public static <T> T match(final String string, final T[] candidates) {
    return match(string, Arrays.asList(candidates));
  }

  /**
   * Return the target constant corresponding to given target declaration among. Return a non-null
   * result, will throw if invalid.
   *
   * @throws RuntimeException If no {@link TargetDecl} is present or if it is invalid.
   */
  public static Target fromDecl(TargetDecl targetDecl) {
    String name = targetDecl.getName();
    return Target.forName(name)
        .orElseThrow(() -> new RuntimeException("Invalid target name '" + name + "'"));
  }

  public void initialize(TargetConfig config) {
    switch (this) {
      case C, CCPP ->
          config.register(
              AuthProperty.INSTANCE,
              BuildCommandsProperty.INSTANCE,
              BuildTypeProperty.INSTANCE,
              ClockSyncModeProperty.INSTANCE,
              ClockSyncOptionsProperty.INSTANCE,
              CmakeIncludeProperty.INSTANCE,
              CompileDefinitionsProperty.INSTANCE,
              CompilerProperty.INSTANCE,
              CoordinationOptionsProperty.INSTANCE,
              CoordinationProperty.INSTANCE,
              DockerProperty.INSTANCE,
              FilesProperty.INSTANCE,
              KeepaliveProperty.INSTANCE,
              NoSourceMappingProperty.INSTANCE,
              PlatformProperty.INSTANCE,
              ProtobufsProperty.INSTANCE,
              SchedulerProperty.INSTANCE,
              SingleThreadedProperty.INSTANCE,
              TracingProperty.INSTANCE,
              TracePluginProperty.INSTANCE,
              VerifyProperty.INSTANCE,
              WorkersProperty.INSTANCE);
      case CPP ->
          config.register(
              BuildTypeProperty.INSTANCE,
              CmakeIncludeProperty.INSTANCE,
              CompilerProperty.INSTANCE,
              DockerProperty.INSTANCE,
              ExportDependencyGraphProperty.INSTANCE,
              ExportToYamlProperty.INSTANCE,
              ExternalRuntimePathProperty.INSTANCE,
              NoRuntimeValidationProperty.INSTANCE,
              PrintStatisticsProperty.INSTANCE,
              Ros2DependenciesProperty.INSTANCE,
              Ros2Property.INSTANCE,
              RuntimeVersionProperty.INSTANCE,
              TracingProperty.INSTANCE,
              WorkersProperty.INSTANCE);
      case Python ->
          config.register(
              AuthProperty.INSTANCE,
              BuildCommandsProperty.INSTANCE,
              BuildTypeProperty.INSTANCE,
              ClockSyncModeProperty.INSTANCE,
              ClockSyncOptionsProperty.INSTANCE,
              CompileDefinitionsProperty.INSTANCE,
              CoordinationOptionsProperty.INSTANCE,
              CoordinationProperty.INSTANCE,
              DockerProperty.INSTANCE,
              FilesProperty.INSTANCE,
              KeepaliveProperty.INSTANCE,
              NoSourceMappingProperty.INSTANCE,
              ProtobufsProperty.INSTANCE,
              SchedulerProperty.INSTANCE,
              SingleThreadedProperty.INSTANCE,
              TracingProperty.INSTANCE,
              TracePluginProperty.INSTANCE,
              WorkersProperty.INSTANCE,
              PythonVersionProperty.INSTANCE);
      case Rust ->
          config.register(
              BuildTypeProperty.INSTANCE,
              CargoDependenciesProperty.INSTANCE,
              CargoFeaturesProperty.INSTANCE,
              ExportDependencyGraphProperty.INSTANCE,
              ExternalRuntimePathProperty.INSTANCE,
              RustIncludeProperty.INSTANCE,
              KeepaliveProperty.INSTANCE,
              RuntimeVersionProperty.INSTANCE,
              SingleFileProjectProperty.INSTANCE,
              SingleThreadedProperty.INSTANCE,
              WorkersProperty.INSTANCE);
      case TS ->
          config.register(
              CoordinationOptionsProperty.INSTANCE,
              CoordinationProperty.INSTANCE,
              DockerProperty.INSTANCE,
              KeepaliveProperty.INSTANCE,
              ProtobufsProperty.INSTANCE,
              RuntimeVersionProperty.INSTANCE);
    }
  }
}
