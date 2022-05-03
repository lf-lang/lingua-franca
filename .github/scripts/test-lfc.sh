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

bin/lfc test/C/src/Minimal.lf

# -c,--clean                         Clean before building.
bin/lfc -c test/C/src/Minimal.lf
bin/lfc --clean test/C/src/Minimal.lf

#    --external-runtime-path <arg>   Specify an external runtime library to
#                                    be used by the compiled binary.

# -f,--federated                     Treat main reactor as federated.
bin/lfc -f test/C/src/Minimal.lf
bin/lfc --federated test/C/src/Minimal.lf

# --rti                              Specify address of RTI.
bin/lfc -f --rti rti test/C/src/Minimal.lf
bin/lfc --federated --rti rti test/C/src/Minimal.lf

# -h,--help                          Display this information.
bin/lfc -h
bin/lfc --help

# -l, --lint                         Enable linting during build.
bin/lfc -l test/Python/src/Minimal.lf
bin/lfc --lint test/Python/src/Minimal.lf

# -n,--no-compile                    Do not invoke target compiler.
bin/lfc -n test/C/src/Minimal.lf
bin/lfc --no-compile test/C/src/Minimal.lf

# -o,--output-path <arg>             Specify the root output directory.
bin/lfc -o . test/C/src/Minimal.lf
bin/lfc --output-path . test/C/src/Minimal.lf

#    --runtime-version <arg>         Specify the version of the runtime
#                                    library used for compiling LF
#                                    programs.
bin/lfc --runtime-version 46a618c01e494b7b476707c30dd6067ad66759d6 test/Cpp/src/Minimal.lf

# -w,--workers                       Specify the default number of worker threads.
bin/lfc -w 2 test/C/src/Minimal.lf
bin/lfc --workers 2 test/C/src/Minimal.lf
bin/lfc --threading true test/C/src/Minimal.lf
bin/lfc --threading false test/C/src/Minimal.lf

#    --target-compiler <arg>         Target compiler to invoke.
# (Added no-compile to avoid adding dependency.)
bin/lfc --target-compiler gcc --no-compile test/C/src/Minimal.lf 

# --version
bin/lfc --version

# Ensure that lfc is robust to symbolic links.
test_with_links "lfc"
