# A handy script that will run a command multiple times
# Usage: run-multiple-times.ps1 command number_of_times
$executable=$args[0]
$number_of_times=$args[1]
for (($k = 0); $k -lt $number_of_times; $k++) {
    Invoke-Expression $executable
}
