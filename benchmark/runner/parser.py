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


def parse_lfc_output(lines):
    times = []
    for line in lines:
        prefix = "---- Elapsed physical time (in nsec): "
        if line.startswith(prefix):
            p = len(prefix)
            ns = int(line[p:].replace(",", ""))
            times.append(ns / 1000000.0)

    return times
