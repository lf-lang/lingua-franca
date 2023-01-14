# LF system tests

**System tests** are complete Lingua Franca programs that are compiled and executed automatically. A test passes if it successfully compiles and runs to completion with normal termination (return code 0). These tests are located in a subdirectory corresponding to their target language.

### Running from the command line

The simplest way to run the regression tests is to use a Bash script called `run-lf-tests` in `$LF/bin`, which takes the target language as a parameter:
```
run-lf-tests C
run-lf-tests Cpp
run-lf-tests Python
run-lf-tests TypeScript
```

You can also selectively run just some of the tests. For example, to run the system tests for an individual target language, do this:
```
./gradlew test --tests org.lflang.tests.runtime.CTest.*
./gradlew test --tests org.lflang.tests.runtime.CppTest.*
./gradlew test --tests org.lflang.tests.runtime.PythonTest.*
./gradlew test --tests org.lflang.tests.runtime.TypeScriptTest.*
```

To run a single test case, use the `runSingleTest` gradle task along with the path to the test source file:
```
./gradlew runSingleTest --args test/C/src/Minimal.lf
```

It is also possible to run a subset of the tests. For example, the C tests are organized into the following categories:

* **generic** tests are `.lf` files located in `$LF/test/C/src`.
* **concurrent** tests are `.lf` files located in `$LF/test/C/src/concurrent`.
* **federated** tests are `.lf` files located in `$LF/test/C/src/federated`.
* **multiport** tests are `.lf` files located in `$LF/test/C/src/multiport`.

To invoke only the tests in the `concurrent` category, for example, do this:
```
cd $LF
./gradlew test --tests org.lflang.tests.runtime.CTest.runConcurrentTests
```


### See also

- [Regression Tests (Handbook)](https://www.lf-lang.org/docs/handbook/regression-tests?target=c)
