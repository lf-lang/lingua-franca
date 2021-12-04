### The Rust code generator

The runtime is hosted over at https://github.com/lf-lang/reactor-rust

LFC generates Cargo projects that pull in this dependency using the git revision number hardcoded in `rust-runtime-version.txt`. You can override this behavior with the `--external-runtime-path` LFC option.

To develop this package and the runtime in sync, it is recommended to clone the runtime repo and set the environment variable `LOCAL_RUST_REACTOR_RT` to the relevant path in your shell. This allows you
- to update the Rust runtime version easily with the script `bin/update-rust-runtime.sh`
- to have your local test runs always link to that local repo instead of using the hardcoded revision. This enables you to e.g. test uncommitted changes to the runtime and also debug them from within CLion.

Note: that variable is not meant to be set in CI, ever.
