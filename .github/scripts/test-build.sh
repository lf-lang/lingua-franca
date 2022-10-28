#!/usr/bin/env bash

# Exit 1 if any command returns with a non-zero exit code.
set -euo pipefail

cd $GITHUB_WORKSPACE

function test_with_links() {
    rm -rf foo
    mkdir -p foo/bar/baz
    ln -s ../bin/${1} foo/link-foo 
    ln -s ../link-foo foo/bar/link-bar
    ln -s ../link-bar foo/bar/baz/link-baz
    foo/bar/baz/link-baz --help
}

# Test the build-lf-cli executable and its flags.
bin/build-lf-cli
bin/build-lf-cli --help
bin/build-lf-cli -h
bin/build-lf-cli -c -o -s
bin/build-lf-cli --clean --offline --stacktrace

# Ensure that build-lf-cli is robust to symbolic links.
test_with_links "build-lf-cli"
