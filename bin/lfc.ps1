#==========================================================
# Description: 	    Run the lfc compiler.
# Authors:          Christian Menard, Peter Donovan
# Usage:            Usage: lfc [options] files...
#==========================================================

$base="$PSScriptRoot\.."
$java_home = "$Env:JAVA_HOME"
$java_cmd = "$java_home\bin\java.exe"
$jarpath_dev="$base\org.lflang.lfc\build\libs\org.lflang.lfc-*-all.jar"
$jarpath_release="$base\lib\jars\org.lflang.lfc-*-all.jar"

function Test-Dev {
    Test-Path "$base\org.lflang.lfc" -PathType container
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
if ([version]$java_version -lt [version]"11.0") {
    throw "JRE $java_version found but 11.0 or greater is required."
}

# invoke lfc
& $java_cmd -jar $(Get-JarPath) $args
