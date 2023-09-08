#============================================================================
# Description:      Build and run the Lingua Franca diagram generator (lfd).
# Authors:          Ruomu Xu
#                   Christian Menard
# Usage:            Usage: lfd-dev [options] files...
#============================================================================


# This script is in $base\bin
$base="$PSScriptRoot\..\"
$gradlew="${base}/gradlew.bat"

# invoke script
& "${gradlew}" --quiet -p "${base}" assemble ":cli:lfd:assemble"
& "${base}/build/install/lf-cli/bin/lfd" @args
