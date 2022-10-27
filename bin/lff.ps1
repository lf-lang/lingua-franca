#==========================================================
# Description: 	    Run the lff compiler.
# Authors:          Ruomu Xu
# Usage:            Usage: lff [options] files...
#==========================================================

$launchScript="$PSScriptRoot\..\lib\scripts\launch.ps1"
# PS requires spattling: https://learn.microsoft.com/en-us/powershell/module/microsoft.powershell.core/about/about_Splatting?view=powershell-7.2
. $launchScript @args
