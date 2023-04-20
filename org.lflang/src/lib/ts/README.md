## How to build against an unpublished reactor-ts
To point the Lingua Franca compiler to a particular development version of `reactor-ts` not yet published on NPM, change the follow line in `package.json`:
```
"@lf-lang/reactor-ts": "semver",
```
where `semver` is some concrete version number, to:
```
git://github.com/lf-lang/reactor-ts.git#ref
```
where `ref` could be a SHA1 hash of a particular commit or a branch name such as `my-feature-branch`.

**NOTE**: the `package.json` in this directory is used as a _default_ only; it can be overridden by a `package.json` provided alongside the LF source file. To make sure that all the tests in CI run with a particular version of the runtime, it is safer to leave this file "as is" and instead adapt `.github/workflows/ts-tests.yml` by adding a `-Druntime="git://github.com/lf-lang/reactor-ts.git#ref"` parameter to the `./gradlew test` command.