#==========================================================
# Description: 	    Run the lfc compiler.
# Authors:          Christian Menardn
# Usage:            Usage: lfc [options] files...
#==========================================================

$base="$PSScriptRoot\.."
$lfbase="$base\org.lflang"
$jarpath="$lfbase\build\libs\org.lflang-1.0.0-SNAPSHOT-all.jar"

# if there is no jar file, then build it first
if (-not (Test-Path $jarpath -PathType leaf)) {
    $old_pwd = $pwd
	cd $base
    ./gradlew generateStandaloneCompiler
	cd $old_pwd
}

$java_home = "$Env:JAVA_HOME"
$java_cmd = "$java_home\bin\java.exe"
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
if ([version]$java_version -lt [version]"8.0") {
    throw "JRE $java_version found but 8.0 or greater is required."
}

# invoke lfc
& $java_cmd -jar $jarpath $args