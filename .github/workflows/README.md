# Lingua Franca workflows

## Continuous Integration
The main CI configuration can be found in [ci.yml](https://github.com/lf-lang/lingua-franca/blob/master/.github/workflows/ci.yml) and gets triggered by pushes to `master` and by pushes to branches involved in an open pull request.

### Benchmark tests

### CLI tests

### Target-specific tests
Each target has its own reusable workflow.

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

**NOTE: [ci.yml](https://github.com/lf-lang/lingua-franca/blob/master/.github/workflows/ci.yml) references these workflows with respect to master (signified by the "@master" suffix). If you edit a workflow and want your changes reflected in the CI run for your pull request, then make sure that the workflow of your feature branch gets invoked instead of the one on master.**

### Unit tests

## Nightly Build
