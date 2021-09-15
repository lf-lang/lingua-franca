# VS Code Extension
Additional details about the VS Code extension can be found at the pull request #376.

## Production Build
It is straightforward to build the Language and Diagram Server. As stated by Alexander:
> Simply run: `gradlew startCode`. You also need `mvn` on your path and if Java 11 is not your default JDK you need to prepend something like this: `JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64/`

If this does not work, it could be because:
* Your Python version does not match the Python version used in `org.lflang.lds/pom.xml`. To resolve this, simply edit `org.lflang.lds/pom.xml` to say "python" instead of "python3", or vice versa.

## Development Build
For development purposes, it is possible to manually perform an incremental build simply by bypassing Maven and Gradle entirely and instead running the Python script `org.lflang.lds/uf.py`. This script will re-compile Java and Kotlin files and add them to the fat jar using the `jar` command with the `-uf` flag. This is far from ideal. If there were an IDE that could build the Language and Diagram Server, development would be easier; however, to my knowledge, there is no such IDE.

The following steps permit one to see the effects of a change to a file or package in about 1 minute as opposed to (say) twenty, thirty, or even forty minutes.
1. Ensure that the appropriate compiler is on your PATH.
  * To build Java files, `javac` is required.
  * To build Kotlin files, [the Kotlin JVM compiler](https://github.com/JetBrains/kotlin/releases/tag/v1.5.30) `kotlinc` is required. It must be the JVM compiler, not the native compiler.
2. Ensure that the language and diagram server fat JAR exists. This file is called `vscode-extension\ls\lflang-lds.jar`.
3. `cd` into the `org.lflang.lds` directory and run the command: ```python3 uf.py <CANONICAL_NAME>``` where <CANONICAL_NAME> is either:
* The canonical name of a package that you would like to update
* The canonical name of a package, followed by the base name of a file that you would like to update, without the extension. For Java files, this corresponds to the canonical name of a class. An example would be: ```python3 uf.py org.lflang.FileConfig```.
4. Open `vscode-extension\src\extension.ts` in Visual Studio Code.
5. Press <kbd>F5</kbd> to run the extension in a new Extension Development Host window.

## Notes
I made some configuration changes that I am not sure are right. These include the following:
1. (Not committed) It was necessary to add the contents of `org.lflang.lds/target/repository` to the Eclipse target by creating a new target based on the default target and adding the entire directory. Go to Window -> Preferences -> Plug-in Development -> Target Platform -> Add... Current target to create a new target based on the default, and then Edit -> Add -> Directory to add the directory from `org.lflang.lds` that has the JARs.
1. (Not committed) It was necessary to add the following dependencies to the MANIFEST.mf of `org.lflang.diagram` to avoid a sneaky-throw error due to a `ClassNotFoundException` from (I believe) `de.cau.cs.kieler.klighd.lsp`. This seems suboptimal since the real source of the dependency is the plugin (which does not have the dependency in its MANIFEST.mf):
  * `org.eclipse.sprotty`,
  * `org.eclipse.sprotty.xtext`,
  * `org.eclipse.sprotty.layout`
1. I added `mvn clean` to the start build process because I observed that changes to the source files were not being reflected in the LDS jar.
1. I explicitly included some jars to the build process via the Python script because they did not seem to be included in the fat jar otherwise (even after efforts to include them appropriately in `build.gradle`, `pom.xml`, and `MANIFEST.mf`).
1. I explicitly ignored some jars in the build process via the Python script because they were conflicting with other jars and apparently could be dispensed with without causing a `ClassDefNotFoundException`.
1. I explicitly allowed some jars to override other jars in the build process via the Python script because they were conflicting with other jars and apparently could **not** be dispensed with without causing a `ClassDefNotFoundException`.
