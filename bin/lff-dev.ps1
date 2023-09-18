#============================================================================
# Description:      Build and run the Lingua Franca code formatter (lff).
# Authors:          Ruomu Xu
#                   Christian Menard
# Usage:            Usage: lff-dev [options] files...
#============================================================================


# This script is in $base\bin
$base="$PSScriptRoot\..\"
$gradlew="${base}/gradlew.bat"

# invoke script
& "${gradlew}" --quiet -p "${base}" assemble ":cli:lff:assemble"
& "${base}/build/install/lf-cli/bin/lff" @args
