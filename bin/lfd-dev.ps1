#============================================================================
# Description:      Build and run the Lingua Franca diagram generator (lfd).
# Authors:          Ruomu Xu
#                   Christian Menard
# Usage:            Usage: lfd-dev [options] files...
#============================================================================


# This script is in $base\bin
$base="$PSScriptRoot\..\"
$gradlew="${base}/gradlew.bat"

# Build only the lfd CLI (not the whole project, which would also build :lsp).
# Try offline first so dev workflows work without network after an initial build.
$buildArgs = @("--quiet", "-p", "${base}", ":cli:lfd:installDist")
& "${gradlew}" @buildArgs --offline
if ($LASTEXITCODE -ne 0) {
    & "${gradlew}" @buildArgs
}
& "${base}/cli/lfd/build/install/lfd/bin/lfd" @args
