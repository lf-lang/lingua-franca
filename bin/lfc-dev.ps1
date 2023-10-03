#============================================================================
# Description:      Build and run the Lingua Franca compiler (lfc).
# Authors:          Ruomu Xu
#                   Christian Menard
# Usage:            Usage: lfc-dev [options] files...
#============================================================================


# This script is in $base\bin
$base="$PSScriptRoot\..\"
$gradlew="${base}/gradlew.bat"

# invoke script
& "${gradlew}" --quiet -p "${base}" assemble ":cli:lfc:assemble"
& "${base}/build/install/lf-cli/bin/lfc" @args
