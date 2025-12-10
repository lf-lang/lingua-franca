# LF integration tests

**Integration tests** are complete Lingua Franca programs that are compiled and executed automatically. A test passes if it successfully compiles and runs to completion with normal termination (return code 0). These tests are located in a subdirectory corresponding to their target language.

### Running from the command line

The simplest way to run integration tests for a specific target is the `targetTest` gradle task. For instance, you can use the following command to run all Rust tests:

```
./gradlew targetTest -Ptarget=Rust
```
You can specify any valid target.

The `targetTest` task is essentially a convenient shortcut for the following:

```
./gradlew core:integrationTest --tests "org.lflang.tests.runtime.<target>Test.*"
```
If you prefer have more control over which tests are executed, you can also use this more verbose version.
Note that the quotation marks are necessary on some shells to prevent the shell from trying to match the `*` to file names.

To run a single test case, use the `singleTest` gradle task along with the path to the test source file:

```
./gradlew singleTest -DsingleTest=test/C/src/Minimal.lf
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

Test categories are declared in the [TestCategory enum in TestRegistry.java](https://github.com/lf-lang/lingua-franca/blob/2611f38cb1e331afbf2fc18f0c9e9ec2758de348/org.lflang.tests/src/org/lflang/tests/TestRegistry.java#L130). Each `.lf` file is identified by the matching containing directory closest to it, or, if there is no such directory, it will be identified as `generic`. E.g., `test/C/src/multiport/Foo.lf` falls in the `multiport` category.

Tests are normally expected to compile without errors and return exit code `0` when executed. Some test categories (e.g., `arduino`) are not attempted to run and are only expected to compile as they might require the presence of particular hardware or exotic software configurations that are not manageable in GitHub Actions, our current platform for Continuous Integration (CI). Only pushes to feature branches associated with an active pull request trigger CI.

### LSP tests

LSP tests run target language tools to lint and find errors in target code such as reaction bodies.
They work by inserting errors into files in this directory, running LFC on them in LSP mode, and
verifying that errors are reported correctly. When a test fails, an error message is printed that
explains why; such an error message will include the string "the expected error could not be found."
The contents of the altered LF file (including the inserted error) are printed, with an arrow `->`
marking the line on which an error message should have been reported.

### See also

- [Regression Tests (Handbook)](https://www.lf-lang.org/docs/handbook/regression-tests?target=c)
