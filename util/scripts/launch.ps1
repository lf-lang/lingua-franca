#==========================================================
# Description: 	    Launch lfc or lff depending on the invoking script.
# Authors:          Christian Menard, Peter Donovan, Ruomu Xu
# Usage:            Usage: launch args...
#                   with invoker with name same as the programme to be invoked.
#==========================================================

# If the invoker is Z:\nkamihara\lf\bin\lfc.ps1, $invokerName will strip out "lfc.ps1" and then get "lfc".
# See https://learn.microsoft.com/en-us/powershell/module/microsoft.powershell.core/about/about_automatic_variables?view=powershell-7.2#myinvocation
$invokerPath = $MyInvocation.PSCommandPath
$invokerName = [System.IO.Path]::GetFileNameWithoutExtension("$(Split-Path -Path $invokerPath -Leaf -Resolve)")

$mainClassTable = @{"lfc" = "org.lflang.cli.Lfc"; "lff" = "org.lflang.cli.Lff"; "lfd" = "org.lflang.cli.Lfd"}
$tool = $null
foreach ($k in $mainClassTable.Keys) {
    if ($invokerName.EndsWith($k)) {
        $tool = $k
        break
    }
}
if ($null -eq $tool) {
    throw ("$invokerName is not a known lf command. Known commands are [$($mainClassTable.Keys)]. " +
          "In case you use a symbolic or hard link to one of the Lingua Franca " +
          "command line tools, make sure that the link's name ends with one of [$($mainClassTable.Keys)]")
}

# This script is in $base\util\scripts
$base="$PSScriptRoot\..\..\"
$gradlew="${base}/gradlew.bat"

# invoke script
if ($args.Length -eq 0) {
    & "${gradlew}" -p "${base}" "cli:${tool}:run"
} else {
    $argsString = $args -join " "
    & "${gradlew}" -p "${base}" "cli:${tool}:run" --args="${argsString}"
}