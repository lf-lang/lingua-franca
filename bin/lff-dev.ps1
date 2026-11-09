#============================================================================
# Description:      Build and run the Lingua Franca code formatter (lff).
# Authors:          Ruomu Xu
#                   Christian Menard
# Usage:            Usage: lff-dev [options] files...
#============================================================================


# This script is in $base\bin
$base="$PSScriptRoot\..\"
$gradlew="${base}/gradlew.bat"

# Build only the lff CLI (not the whole project, which would also build :lsp).
# Try offline first so dev workflows work without network after an initial build.
$buildArgs = @("--quiet", "-p", "${base}", ":cli:lff:installDist")
& "${gradlew}" @buildArgs --offline
if ($LASTEXITCODE -ne 0) {
    & "${gradlew}" @buildArgs
}
& "${base}/cli/lff/build/install/lff/bin/lff" @args
