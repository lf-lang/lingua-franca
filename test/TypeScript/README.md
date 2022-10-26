# Library of TypeScript tests
To run the entire test suite, execute `./gradlew test --tests org.lflang.tests.runtime.TypeScriptTest.*`.

## Using an alternative runtime
To run the tests with an alternative runtime, use the `-Druntime` flag to specify where to find it.

### Examples
- To use a local checkout of `reactor-ts` located in the local file system in the directory `~/lf-lang/reactor-ts`:
```
./gradlew test --tests org.lflang.tests.runtime.TypeScriptTest.* -Druntime="~/lf-lang/reactor-ts"
```
- To point to a particular ref (e.g., `feature-branch`) of the `reactor-ts` repository:
```
./gradlew test --tests org.lflang.tests.runtime.TypeScriptTest.* -Druntime="git://github.com/lf-lang/reactor-ts.git#feature-branch"
```
- Note that `lfc` can be pointed to an alternative runtime as well, using the `external-runtime-path` switch:
```
lfc test/TypeScript/src/Minimal.lf --external-runtime-path ~/lf-lang/reactor-ts
```