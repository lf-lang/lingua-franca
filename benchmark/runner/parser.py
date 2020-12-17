def parse_akka_output(lines):
    times = []
    for line in lines:
        line_split = line.split()
        if "Iteration-" in line and len(line_split) == 4 and line_split[3] == "ms":
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
