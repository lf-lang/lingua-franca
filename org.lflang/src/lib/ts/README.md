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
