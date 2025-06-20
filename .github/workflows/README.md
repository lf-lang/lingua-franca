# Lingua Franca workflows

## Continuous Integration
The main two CI configurations can be found in [all-misc.yml](https://github.com/lf-lang/lingua-franca/blob/master/.github/workflows/all-misc.yml) and [all-targets.yml](https://github.com/lf-lang/lingua-franca/blob/master/.github/workflows/all-targets.yml). The [all-misc.yml](https://github.com/lf-lang/lingua-franca/blob/master/.github/workflows/all-misc.yml) workflow runs the non-target specific tests, and [all-targets.yml](https://github.com/lf-lang/lingua-franca/blob/master/.github/workflows/all-targets.yml) workflow runs the target specific tests. They get triggered by pushes to `master` and by pushes to branches involved in an open pull request.

### Benchmark tests
Benchmarks in the [benchmarks-lingua-franca repo](https://github.com/lf-lang/benchmarks-lingua-franca) are also tested.

### CLI tests
The [lfc-tests.yml](https://github.com/lf-lang/lingua-franca/blob/master/.github/workflows/lfc-tests.yml) workflow tests command line access to the Lingua Franca compiler via `lfc`.
### Target-specific tests
Each target has its own [reusable workflow](https://docs.github.com/en/actions/learn-github-actions/reusing-workflows).
#### C/CCpp ([c-tests.yml](https://github.com/lf-lang/lingua-franca/blob/master/.github/workflows/c-tests.yml))
This workflow has the following (optional) arguments:
- `compiler-ref` to specify which ref of the `lingua-franca` repository to check out; and
- `runtime-ref` to specify which ref of the `reactor-c` submodule to check out.

#### C++ ([cpp-tests.yml](https://github.com/lf-lang/lingua-franca/blob/master/.github/workflows/cpp-tests.yml))

#### Python ([py-tests.yml](https://github.com/lf-lang/lingua-franca/blob/master/.github/workflows/py-tests.yml))
This workflow has the following (optional) arguments:
- `compiler-ref` to specify which ref of the `lingua-franca` repository to check out;
- `reactor-c-ref` to specify which ref of the `reactor-c` submodule to check out; and
- `reactor-c-py-ref` to specify which ref of the `reactor-c-py` submodule to check out.

#### Rust ([rs-tests.yml](https://github.com/lf-lang/lingua-franca/blob/master/.github/workflows/rs-tests.yml))

#### TypeScript ([ts-tests.yml](https://github.com/lf-lang/lingua-franca/blob/master/.github/workflows/ts-tests.yml))
This workflow has the following (optional) arguments:
- `compiler-ref` to specify which ref of the `lingua-franca` repository to check out; and
- `runtime-ref` to specify which ref of the `reactor-ts` submodule to check out.

### Unit tests
Several parts of the compiler are probed using unit tests. These tests are carried out using the [unit-tests.yml](https://github.com/lf-lang/lingua-franca/blob/master/.github/workflows/unit-tests.yml) workflow. This workflow also collects code coverage statistics based on these unit tests _as well as_ the joint coverage achieved by the target-specific integration tests. It does this using an [extra JUnit test](https://github.com/lf-lang/lingua-franca/blob/master/org.lflang.tests/src/org/lflang/tests/compiler/CodeGenCoverage.java) that carries out a lighter version of all the integration tests (skipping the target compilation and program execution parts, which do not involve much of our own compiler code).

### Utilities
Satellite repositories that make use of Lingua Franca may want to reuse workflows that are of general utility.
#### Extract a ref from a file ([extract-ref.yml](https://github.com/lf-lang/lingua-franca/blob/master/.github/workflows/extract-ref.yml))
If a repository has a text file that stores a ref (e.g., a SHA1 hash of a commit) and its contents must be used as a variable in a workflow, then [extract-ref.yml](https://github.com/lf-lang/lingua-franca/blob/master/.github/workflows/extract-ref.yml) can be used to accomplish this. This workflow takes a single (required) parameter:
 - `file`: a string that specifies the path to the text file that has the ref.
After workflow execution, the value of the output `ref` will be equal to the first line in the given `file`.
## Nightly Build
See [nightly-build.yml](https://github.com/lf-lang/lingua-franca/blob/master/.github/workflows/nightly-build.yml).

## Debugging tests

To debug test failures that are difficult to reproduce locally, it can be useful
to add a step such as [this one](https://github.com/marketplace/actions/debugging-with-ssh) to SSH into the GitHub Actions runner. Such a debugging step
should not be included in the `master` version of the workflow file.
