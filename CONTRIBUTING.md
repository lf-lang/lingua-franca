# Contributing to the Lingua Franca compiler

> [!IMPORTANT]
> This document contains guidelines specific to this repository. Please also read the general [contributing guidelines](https://github.com/lf-lang/.github/blob/main/CONTRIBUTING.md) of the lf-lang organization.

This repository hosts the source code for the Lingua Franca compiler, which is written in Java and Kotlin.
We welcome contributions from the community and provide these guidelines in order to make contributing a smooth, efficient, and fun process.

## Contributing code

You can work on the Lingua Franca code base in your favorite editor and build [using Gradle](https://www.lf-lang.org/docs/developer/downloading-and-building#building-the-command-line-tools). We recommend to use the [IntelliJ](https://www.lf-lang.org/docs/handbook/intellij) IDE.

### Writing tests

An integral part of contributing code is writing tests.

**Unit tests** for the compiler are located in the `org.lflang.tests` package and are implemented using the JUnit test framework. These tests are invoked using Gradle. For example, to run all the tests in the `org.lflang.tests.compiler` package, use the following command:
```
./gradlew test --tests org.lflang.tests.compiler.*
```
**Integration tests** consist of complete Lingua Franca programs that are located in the `test` directory in the root of the repository and are organized first by target, then by category. These tests get indexed automatically and are also invoked using Gradle. For instance, to run all the tests for the C target, use the following command:
```
 ./gradlew test --tests org.lflang.tests.runtime.CTest.*
 ```
Test categories are declared in the [TestCategory enum in TestRegistry.java](https://github.com/lf-lang/lingua-franca/blob/2611f38cb1e331afbf2fc18f0c9e9ec2758de348/org.lflang.tests/src/org/lflang/tests/TestRegistry.java#L130). Each `.lf` file is identified by the matching containing directory closest to it, or, if there is no such directory, it will be identified as `generic`. E.g., `test/C/src/multiport/Foo.lf` falls in the `multiport` category. Tests are normally expected to compile without errors and return exit code `0` when executed. Some test categories (e.g., `arduino`) are not attempted to run and are only expected to compile as they might require the presence of particular hardware or exotic software configurations that are not manageable in GitHub Actions, our current platform for Continuous Integration (CI). Only pushes to [feature branches](#feature-branches) associated with an active [pull request](#pull-requests) trigger CI.

### Workflow
All code contributions must go through a [pull request (PR)](https://github.com/lf-lang/.github/blob/main/CONTRIBUTING.md#pull-requests), pass all tests run in CI, and get an approving review before it is merged. Pushing to the `master` branch is restricted. All code review is conducted using the Github review system on PRs. Before requesting a code review, ensure that you have:
- applied the [code formatter](#code-style-and-formatting);
- [documented](#code-style-and-formatting) your code;
- written [tests](#writing-tests) that cover your code; and
- accompanied any remaining `TODO`s or `FIXME`s with a link to an active issue.


### Code style and formatting
The Lingua Franca compiler is implemented in Java and Kotlin. The overarching advice is to use each language's most widely used idioms and conventions, which are fortunately very similar. The code base is shipped with a [Spotless](https://github.com/diffplug/spotless) configuration to check and enforce style compliance. Lingua Franca code (e.g., tests) in this repository is also automatically formatted via Spotless.


#### Formatting

- To check that modified files are formatted correctly, run:

```
./gradlew spotlessCheck
```

- To apply the changes recommended by the formatter, run:

```
./gradlew spotlessApply
```

_Java code formatting in IntelliJ_

Follow the directions [here](https://github.com/ragurney/spotless-intellij-gradle) to install the Spotless Gradle plugin. We are not currently aware of a way to run Spotless
automatically on file save, but it is possible to configure a keybinding.

There is a [plugin](https://github.com/google/google-java-format) specifically for enforcing Google's style guide, but
we have found difficulty getting it to work properly.

_Lingua Franca code formatting in VS Code_

Formatting is provided by the Lingua Franca VS Code extension.

You can enable formatting on save by going to `Settings` and searching for "format" and checking the box
`Editor: Format on Save`. This can be either a user-level or workspace-level setting.

_Checking using Git hooks_

If you prefer not to use formatting on save but would still like to avoid committing unformatted code, you can add a
file `.git/hooks/pre-commit` with the contents:

```
./gradlew spotlessCheck
```

#### General guidelines
- _Do not copy-paste code._ If you want to reuse code, factor it out into a method and call it.
- _Keep methods concise._ As a rule of thumb, a method should fit on your screen so that it can be read without scrolling. We impose no hard limit on method length, but anything above 40 lines should be considered for breaking up.
- _Keep classes concise._ Classes should be limited in scope and not become too large. Anything above 1000 lines probably needs refactoring.
- _Do not leave FIXMEs_. Unaddressed `FIXME`s should not be allowed to pass code review unless they are accompanied with a link to a GitHub issue.

_Comments_

Please adhere to the following principles when writing documentation for your code:
- Write descriptions in English.
- Do not use contractions like "aren't" or "isn't".
- Use imperative in the description of a method, i.e., write "Compute the shortest path," not "Computes the shortest path" (the latter is not a complete sentence).
- In `@param` JavaDoc tag descriptions it is OK to use incomplete sentences in the interest of brevity.

#### Java-specific guidelines

We use the [Google Java style guide](https://google.github.io/styleguide/javaguide.html), which leaves a few things unspecified that we discuss here in a bit more depth.

_Ordering of class contents ([S3.4.2](https://google.github.io/styleguide/javaguide.html#s3.4.2-ordering-class-contents))_

> The order you choose for the members and initializers of your class can have a great effect on learnability. However, there's no single correct recipe for how to do it; different classes may order their contents in different ways.

> What is important is that each class uses some logical order, which its maintainer could explain if asked. For example, new methods are not just habitually added to the end of the class, as that would yield "chronological by date added" ordering, which is not a logical ordering.

When in doubt, the following suggestions might help:
1. Keep all fields at the very top (and sort them internally by visibility).
2. After fields, declare constructors.
3. After constructors, put all methods, in "some logical order".
4. Put inner classes at the very end.

_Visibility_

Avoid public non-final fields and public collection fields (making them `final` does not make their contents immutable).

_Inheritance_

Design for inheritance or prohibit it. Make new classes final by default, and avoid using protected members. The class should only be unsealed if a use case for inheritance shows up, which is in many cases never. Often, composition can offer a better solution than inheritance (also see a [blog post](https://matthiasnoback.nl/2018/09/final-classes-by-default-why/) on this).

_Collections_

Use interfaces like `List`, `Set`, `Map` to type variables, constructor parameters, and method signatures, not concrete implementations like `LinkedList`, `HashSet`, or `LinkedHashMap`.

_Collection mutability_

In Java, instances of collection classes may be read-only. They will throw an exception when modification is attempted. Read-only views on collections are used to prevent external code from modifying an internal collection by accident, which may break class invariants. Encapsulating modifications to the collection into extra mutator methods is good practice, because it allows maintaining class invariants if the class evolves.

- Assume that any list returned from a method of an object that you do not control is **unmodifiable unless otherwise documented**. That means, make a copy if you need a local modification.

- If you return a collection which is a field of the current object, **make it unmodifiable** (i.e., `return Collections.unmodifiableList(internalList);`). If the API *must* return a modifiable list, then document that it is modifiable. (It's usually better to provide mutator methods to encapsulate the modification to the collection.)

```
/* Assume the returned list is unmodifiable */
List<String> contents = container.getListOfContents();
/* You can iterate on the list, get an item, but not set/add/remove. To make local modifications, make a copy:*/
contents = new ArrayList<>(contents);
/* Now the list is modifiable, but changes do not affect the `container` object */
contents.add("extra");

/* If you need to modify the internal list of the `container`, add a method to the container that encapsulates that logic */
container.add("extra");
```

_JavaDoc_

The [Google style guide](https://google.github.io/styleguide/javaguide.html#s7-javadoc) is pretty terse about JavaDoc, in particular about [where JavaDoc is required](https://google.github.io/styleguide/javaguide.html#s7.3-javadoc-where-required). Please refer to the [Oracle JavaDoc style guide](https://www.oracle.com/technical-resources/articles/java/javadoc-tool.html#styleguide) for detailed guidelines.

_Compact JavaDoc_

Omit `@param` and `@return` tags from JavaDoc if the summary line already describes everything there is to know about them.
For instance, prefer:

```
/** Return the sum of a and b. */
public static int getSum(int a, int b) { ... }
```
over
```
/**
 * Return the sum of two integers.
 * @param a an integer
 * @param b an integer
 * @return the sum of both parameters
 */
public static void printSum(int a, int b) { ... }
```

Additional information about "compact style" JavaDoc can be found [here](https://www.cs.cornell.edu/courses/JavaAndDS/JavaStyle.html#Comments).

#### Kotlin-specific guidelines

Please follow the [Kotlin coding conventions](https://kotlinlang.org/docs/coding-conventions.html).
