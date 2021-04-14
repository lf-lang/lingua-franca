#==========================================================
# Description:  Run all tests for a given LF target.
# Author:       Christian Menard
# Usage:        run-lf-tests [TARGET]
#==========================================================

$base="$PSScriptRoot\.."

if ($args.count -ne 1) {
    Write-Host "Usage: run-lf-tests [target]"
} else {
    $old_pwd = $pwd
	cd $base
	& .\gradlew.bat clean test --tests "org.lflang.tests.runtime.${args[0]}Test"
	cd $old_pwd
}