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

# Test the build-lfc executable and its flags.
bin/build-lfc
bin/build-lfc --help
bin/build-lfc -h
bin/build-lfc --run --help
bin/build-lfc -r --help
bin/build-lfc --run test/C/src/Minimal.lf
bin/build-lfc -c -o -s
bin/build-lfc --clean --offline --stacktrace

# Ensure that build-lfc is robust to symbolic links.
test_with_links "build-lfc"

bin/lfc --help
bin/lfc test/C/src/Minimal.lf

# FIXME: add more tests here

# Ensure that lfc is robust to symbolic links.
test_with_links "lfc"
