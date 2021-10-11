#!/bin/bash
# This script updates the version of the Rust runtime used by
# LFC and tested in CI. See README.md in Rust code generator
# directory.

if [[ -z "$LOCAL_RUST_REACTOR_RT" ]]; then
 echo "Set LOCAL_RUST_REACTOR_RT properly plz"
 exit 1
fi

cd "$LOCAL_RUST_REACTOR_RT" || (echo "could not cd to $LOCAL_RUST_REACTOR_RT" && exit 1)
revision_no="$(git rev-parse HEAD)"
cd -

# the directory of the script (LF_ROOT/bin)
script_dir="$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"


echo "$revision_no" > "$script_dir/../org.lflang/src/org/lflang/generator/rust/rust-runtime-version.txt"

echo "updated to $revision_no"
