#============================================================================
# Description:      Build and run the Lingua Franca compiler (lfc).
# Authors:          Ruomu Xu
#                   Christian Menard
# Usage:            Usage: lfc-dev [options] files...
#============================================================================


# This script is in $base\bin
$base="$PSScriptRoot\..\"
$gradlew="${base}/gradlew.bat"

# Build only the lfc CLI (not the whole project, which would also build :lsp).
# Try offline first so dev workflows work without network after an initial build.
$buildArgs = @("--quiet", "-p", "${base}", ":cli:lfc:installDist")
& "${gradlew}" @buildArgs --offline
if ($LASTEXITCODE -ne 0) {
    & "${gradlew}" @buildArgs
}
& "${base}/cli/lfc/build/install/lfc/bin/lfc" @args
