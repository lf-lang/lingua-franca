#!/usr/bin/env bash

# Exit 1 if any command returns with a non-zero exit code.
set -euo pipefail

cd $GITHUB_WORKSPACE

# just a couple of smoke tests
bin/lff --help
bin/lff --version

bin/lff -d test/C/src/Minimal.lf
bin/lff --dry-run test/Cpp/src/Minimal.lf

bin/lff test/C/src/Minimal.lf
bin/lff test/Cpp/src/Minimal.lf
