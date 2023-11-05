# LF integration tests

**Integration tests** are complete Lingua Franca programs that are compiled and executed automatically. A test passes if it successfully compiles and runs to completion with normal termination (return code 0). These tests are located in a subdirectory corresponding to their target language.

### Running from the command line

The simplest way to run integration tests for a specific target is the `targetTest` gradle task. For instance, you can use the following command to run all Rust tests:
```
./gradlew targetTest -Ptarget=Rust
```
You can specify any valid target. If you run the task without specifying the target property, `./gradlew targetTest` will produce an error message and list all available targets.

The `targetTest` task is essentially a convenient shortcut for the following:
```
./gradlew core:integrationTest --tests org.lflang.tests.runtime.<target>Test.*
```
If you prefer have more control over which tests are executed, you can also use this more verbose version.

On Zsh (Z shell), which is the default shell for macOS, make sure to add
quotes if `*` is used to prevent Zsh from matching the test name against the
filesystem and returning a `zsh: no matches found` error.
```
./gradlew core:integrationTest --tests "org.lflang.tests.runtime.<target>Test.*"
```

It is also possible to run a subset of the tests. For example, the C tests are organized into the following categories:

* **generic** tests are `.lf` files located in `$LF/test/C/src`.
* **concurrent** tests are `.lf` files located in `$LF/test/C/src/concurrent`.
* **federated** tests are `.lf` files located in `$LF/test/C/src/federated`.
* **multiport** tests are `.lf` files located in `$LF/test/C/src/multiport`.

To invoke only the C tests in the `concurrent` category, for example, do this:
```
./gradlew core:integrationTest --tests org.lflang.tests.runtime.CTest.runConcurrentTests
```

To run a single test case, use the `singleTest` gradle task along with the path to the test source file:
```
./gradlew singleTest -DsingleTest=test/C/src/Minimal.lf
```

### LSP tests

LSP tests run target language tools to lint and find errors in target code such as reaction bodies.
They work by inserting errors into files in this directory, running LFC on them in LSP mode, and
verifying that errors are reported correctly. When a test fails, an error message is printed that
explains why; such an error message will include the string "the expected error could not be found."
The contents of the altered LF file (including the inserted error) are printed, with an arrow `->`
marking the line on which an error message should have been reported.

### See also

- [Regression Tests (Handbook)](https://www.lf-lang.org/docs/handbook/regression-tests?target=c)
