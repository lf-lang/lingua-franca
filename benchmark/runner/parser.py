import dataclasses
import statistics


# A record to store benchmark results
@dataclasses.dataclass
class Times:
    best: float = None
    worst: float = None
    median: float = None


def parse_akka_output(lines):
    times = []
    for line in lines:
        if "Iteration-" in line:
            times.append(float(line.split()[2]))

    return Times(best=min(times), worst=max(times), median=statistics.median(times))


def parse_lfcpp_output(lines):
    best = None
    worst = None
    median = None
    for line in lines:
        if "Best Time:" in line:
            best = float(line.split()[2])
        if "Worst Time:" in line:
            worst = float(line.split()[2])
        if "Median:" in line:
            median = float(line.split()[1])

    return Times(best=best, worst=worst, median=median)
