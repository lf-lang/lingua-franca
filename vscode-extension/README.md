# VS Code Extension

## Production Build
The extension is built and installed by the Gradle task "startCode":
> ./gradlew startCode

The installation process requires that the following are true:
* Python 3 (`python3`) is installed. (Currently, the workaround if Python is invoked by `python` instead of `python3` on your machine is to edit `org.lflang.lds/pom.xml` to specify `python` in place of `python3`.)
* VS Code (`code`) is installed.
* Maven (`mvn`) is installed.
* Java 11 is your default JDK (specified by the JAVA\_HOME environment variable)
* You have deleted the untracked directories `org.lflang/src-gen` and `org.lflang/xtend-gen`, which
may contain generated code from a different branch.

This build process begins and ends with Gradle tasks, but it also depends on Tycho for collecting dependencies and a Python script for creating a fat jar with those dependencies.

## Development Build
For development purposes, it is possible to manually perform an incremental build simply by bypassing Maven and Gradle entirely and instead running the Python script `org.lflang.lds/uf.py`. This script will re-compile Java and Kotlin files and add them to the fat jar using the `jar` command with the `-uf` flag.

This is not ideal. If there were an IDE that could build the Language and Diagram Server, then development would be easier. However, there is no such IDE.

1. Ensure that the appropriate compiler is on your PATH.
  * To build Java files, `javac` is required.
  * To build Kotlin files, [the Kotlin JVM compiler](https://github.com/JetBrains/kotlin/releases/tag/v1.5.30) `kotlinc` is required. It must be the JVM compiler, not the native compiler.
2. Ensure that the language and diagram server fat JAR exists. This file is called `vscode-extension\ls\lflang-lds.jar`. If it does not exist, then it is necessary to build it using the build task: `./gradlew startCode`.
3. `cd` into the `org.lflang.lds` directory and run the command: ```python3 uf.py <CANONICAL_NAME>``` where <CANONICAL_NAME> is either:
* the canonical name of a package that you would like to update, or
* the canonical name of the class that you would like to update. An example would be: ```python3 uf.py org.lflang.FileConfig```. This will also update any nested classes, and it should work as you would expect even for Kotlin files that do not include exactly one top-level class.
4. Open `vscode-extension\src\extension.ts` in Visual Studio Code.
5. Press <kbd>F5</kbd> to run the extension in a new Extension Development Host window.

