#==========================================================
# Description: 	    Run the lfc compiler.
# Authors:          Ruomu Xu
# Usage:            Usage: lfc [options] files...
#==========================================================

$launchScript="$PSScriptRoot\..\lib\scripts\launch.ps1"
# PS requires spattling: https://learn.microsoft.com/en-us/powershell/module/microsoft.powershell.core/about/about_Splatting?view=powershell-7.2
. $launchScript @args
