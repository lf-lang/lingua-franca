# Library of TypeScript tests
To run the entire test suite, execute `./gradlew test --tests org.lflang.tests.runtime.TypeScriptTest.*`.

## Using an alternative runtime
To run the tests with an alternative runtime, use the `-Druntime` flag to specify where to find it.

### Examples
- To use a local checkout of `reactor-ts` located in the local file system in the directory `~/lf-lang/reactor-ts`:
```
./gradlew test --tests org.lflang.tests.runtime.TypeScriptTest.* -Druntime="~/lf-lang/reactor-ts"
```
- Note that `lfc` can be pointed to an alternative runtime as well, using the `external-runtime-path` switch:
```
lfc test/TypeScript/src/Minimal.lf --external-runtime-path ~/lf-lang/reactor-ts
```
- To point `lfc` to a particular ref (e.g. `main`, `v0.1.0` or `f8c6d2379f278e22ad48410bf06cf0909405ecc3`) in the `lf-lang/reactor-ts` repo:
```
lfc test/TypeScript/src/Minimal.lf --runtime-version <ref>
```