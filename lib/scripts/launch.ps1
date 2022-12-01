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

$mainClassTable = @{"lfc" = "org.lflang.cli.Lfc"; "lff" = "org.lflang.cli.Lff"}
$mainClassName = $null
foreach ($k in $mainClassTable.Keys) {
    if ($invokerName.EndsWith($k)) {
        $mainClassName = $mainClassTable[$k]
        break
    }
}
if ($null -eq $mainClassName) {
    throw ("$invokerName is not a known lf command. Known commands are [$($mainClassTable.Keys)]. " +
          "In case you use a symbolic or hard link to one of the Lingua Franca " +
          "command line tools, make sure that the link's name ends with one of [$($mainClassTable.Keys)]")
}

# This script is in $base\lib\scripts
$base="$PSScriptRoot\..\..\"
$java_home = "$Env:JAVA_HOME"
$java_cmd = "$java_home\bin\java.exe"
$jarpath_dev="$base\org.lflang\build\libs\org.lflang-*.jar"
$jarpath_release="$base\lib\jars\org.lflang-*.jar"

function Test-Dev {
    Test-Path "$base\org.lflang" -PathType container
}

function Get-JarPath {
    if (Test-Dev) {
        if (Test-Path $jarpath_dev -PathType leaf) {
            $jarpath=$(Get-ChildItem $jarpath_dev).toString()
        } else {
            throw "Failed to find a copy of the Lingua Franca compiler matching the pattern ""$jarpath_dev"". Did you remember to build?"
        }
    } else {
        if (Test-Path $jarpath_release -PathType leaf) {
            $jarpath=$(Get-ChildItem $jarpath_release).toString()
        } else {
            throw "Failed to find a copy of the Lingua Franca compiler matching the pattern ""$jarpath_release""."
        }
    }
    $jarpath
}

# check if we can find java executable in $java_home
if (-not (Test-Path $java_cmd)) {
    # otherwise, try to run java directly
	if (-not (Get-Command java -errorAction SilentlyContinue)) {
	    throw "JRE not found"
	}
    $java_cmd = "java"
}

# check for correct java version
$java_version = (Get-Command java | Select-Object -ExpandProperty Version).toString()
if ([version]$java_version -lt [version]"17.0") {
    throw "JRE $java_version found but 17.0 or greater is required."
}

# invoke lff
& $java_cmd -classpath $(Get-JarPath) $mainClassName $args
