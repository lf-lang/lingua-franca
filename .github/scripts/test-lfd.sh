#!/usr/bin/env bash

# Exit 1 if any command returns with a non-zero exit code.
set -euo pipefail

cd $GITHUB_WORKSPACE

function test_with_links() {
    rm -rf foo
    mkdir -p foo/bar/baz
    ln -s ../bin/${1} foo/link-foo
    ln -s ../link-foo foo/bar/link-bar
    ln -s ../link-bar foo/bar/baz/link-${1}
    foo/bar/baz/link-${1} --help
}

bin/lfd-dev test/C/src/Minimal.lf

# Ensure that lfd can be invoked via symbolic links.
test_with_links "lfd-dev"

# Ensure that lfd can be invoked from outside the root directory.
cd bin
./lfd-dev --help
cd ..
