# VS Code Extension
It is straightforward to build the Language and Diagram Server. As stated by Alexander:
> Simply run: `gradlew startCode`. You also need `mvn` on your path and if Java 11 is not your default JDK you need to prepend something like this: `JAVA_HOME=/usr/lib/jvm/java-11-openjdk-amd64/`

If this does not work, it could be because:
* Your operating system is Windows. The simplest solution to this issue is not to use Windows. However, if you must use windows, you will need to:
  * Prefix any Gradle Exec commands with 'cmd', '\c'. For example, if the content of an Exec command is "commandLine 'mvn', 'clean'", then it will become "commandLine 'cmd', '\c', 'mvn', 'clean'
  * Make sure that the absolute path to the repository "lingua-franca" is at most 14 characters long so that you do not reach the 260-character limit.
* Your Python version does not match the Python version used in `org.lflang.lds/pom.xml`. To resolve this, simply edit `org.lflang.lds/pom.xml` to say "python" instead of "python3", or vice versa.

Additional details about the VS Code extension can be found at the pull request #376.

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
