import re
# Parse the standard output of each target to figure out the elapsed time for an iteration
# For LF targets, the output format is defined by target-specific BenchmarkRunner reactors
# Each of these functions must return a list of floats corresponding to the time in _milliseconds_

def parse_akka_output(lines):
    times = []
    for line in lines:
        line_split = line.split()
        if "Iteration-" in line and len(line_split) == 4 and line_split[3] == "ms":
            times.append(float(line.split()[2]))
    return times


def parse_caf_output(lines):
    times = []
    for line in lines:
        line_split = line.split()
        if "iteration-" in line and len(line_split) == 4 and line_split[3] == "ms":
            times.append(float(line.split()[2]))
    return times


def parse_lfcpp_output(lines):
    times = []
    for line in lines:
        line_split = line.split()
        if (
            "Iteration " in line
            and len(line_split) == 5
            and line_split[2] == "-"
            and line_split[4] == "ms"
        ):
            times.append(float(line.split()[3]))
    return times


parse_lfc_output = parse_lfcpp_output


def parse_lf_rust_output(lines):
    times = []
    for line in lines:
        match = re.search(r'Iteration (\d+)\t- (\d+) ms\t= (\d+) µs\t= (\d+) ns', line)
        if match:
            times.append(float(match.group(4)) / 1000000.0)
    return times
