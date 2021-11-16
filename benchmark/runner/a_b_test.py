#!/usr/bin/env python3

import argparse
from datetime import datetime
import numpy as np
import os
import sys
import csv

from run_benchmark import execute_command
from parser import parse_lfc_output  # TODO: Generalize to other targets.

TYPICAL_BENCHMARK_TIME_MILLISECONDS = 1000 * 3

def get_benefit(desirable_samples):
    def benefit(a_times_ms, b_times_ms):
        """Returns an estimate of the benefit of doing one more sample. When the benefit
        seems rather low, the estimate is 1.
        """
        n = min(len(a_times_ms), len(b_times_ms))
        return (desirable_samples / n) ** 2  # This is a blind heuristic.
    return benefit

def cost(a_times_ms, b_times_ms):
    """Returns an estimate of the cost of doing one more sample. When the cost
    seems rather high, the estimate is 1.
    """
    return np.mean(a_times_ms + b_times_ms) / TYPICAL_BENCHMARK_TIME_MILLISECONDS

def change(a_times_ms, b_times_ms):
    """Returns the change in mean execution time relative to a_times_ms."""
    a_mean = np.mean(a_times_ms)

    return (np.mean(b_times_ms) - a_mean) / a_mean

def summarize(benchmark, rng, a_times_ms, b_times_ms, n_boot=10000):
    """Returns a comparison of a_times_ms and b_times_ms in the form of a dictionary
    containing the following entries:
     - "benchmark": The name of the benchmark.
     - "n1": The number of trials used for the "main" version.
     - "n2": The number of trials used for the "feature" version.
     - "change": The change in mean execution time relative to a_times_ms.
     - "significant95": Whether the difference is statistically significant at a 95%
     confidence level.
     - "p": The computed two-tailed p-value.
    """
    all = list(a_times_ms) + list(b_times_ms)
    indices = list(range(len(all)))
    def boot():
        selected_a = set(rng.choice(indices, len(a_times_ms)))
        return (
            # This represents an alternative value of a_times_ms.
            [all[i] for i in selected_a],
            # This represents an alternative value of b_times_ms.
            [all[i] for i in indices if i not in selected_a],
        )
    alternative_changes = [
        change(a_times, b_times) for a_times, b_times in [boot() for _ in range(n_boot)]
    ]
    actual_change = change(a_times_ms, b_times_ms)
    p = sum(
        abs(alt) > abs(actual_change) for alt in alternative_changes
    ) / len(alternative_changes)
    return {
        "benchmark": benchmark,
        "n1": len(a_times_ms),
        "n2": len(b_times_ms),
        "change": actual_change,
        "significant95": p < 0.05,
        "p": p
    }


def main_files(a, b, rng, results, summary, benefit):
    """Do A/B testing on the executables a (main) and b (feature)."""
    for exe in (a, b):
        assert os.access(exe, os.X_OK), "{} is not executable".format(exe)
    benchmark = os.path.splitext(os.path.basename(a))[0]
    a_times_ms = []
    b_times_ms = []
    print("Running versions of '{}'...".format(benchmark), end=" ")
    while (
        not a_times_ms or not b_times_ms
        or benefit(a_times_ms, b_times_ms) > cost(a_times_ms, b_times_ms)
    ):
        exe, times, version = (
            (a, a_times_ms, "main") if rng.random() > 0.5
            else (b, b_times_ms, "feature")
        )
        output, code = execute_command([exe])
        if code:
            print("Execution of '{}' terminated with error code {}. Output:\n{}".format(
                exe, code, output
            ))
        else:
            new_times = parse_lfc_output(output)
            times += new_times
            for time_ms in new_times:
                print("{}{}".format(version, round(time_ms)), end=" ")
                sys.stdout.flush()
                results.writerow({
                    "benchmark": benchmark, "version": version, "time_ms": time_ms
                })
    print()
    summary.writerow(summarize(benchmark, rng, a_times_ms, b_times_ms))

def main_dirs(a, b, rng, results, summary, benefit):
    """Do A/B testing on the directories a (main) and b (feature)."""
    a_files = set(os.listdir(a))
    b_files = set(os.listdir(b))
    pairs = [("main", a_files), ("feature", b_files)]
    for name0, files0, name1, files1 in [
        (*tup, *tup) for tup, tup in zip(pairs, reversed(pairs))
    ]:
        missing = [f for f in files0 if f not in files1]
        if any(missing):
            print("Omitting '{}' in '{}' because '{}' does not "
                "contain {}.".format(
                    ", ".join(missing), name0, name1,
                    "them" if len(missing) > 1 else "it"
                ))
    for exe in sorted(list(a_files.intersection(b_files))):
        main_files(
            os.path.join(a, exe), os.path.join(b, exe),
            rng, results, summary, benefit
        )

def set_directory():
    """Creates a directory in the 'outputs' folder for outputs and sets it as the
    current working directory.
    """
    dir = os.path.join(
        "outputs",
        datetime.today().strftime("%Y-%m-%d"),
        datetime.now().strftime("%H-%M-%S")
    )
    os.makedirs(dir)
    os.chdir(dir)

def main(args):
    a = os.path.abspath(args.a)
    b = os.path.abspath(args.b)
    assert os.path.exists(a), "{} does not exist.".format(a)
    assert os.path.exists(b), "{} does not exist.".format(b)
    set_directory()
    with open("results.csv", "w", newline="") as results_file, \
            open("summary.csv", "w", newline="") as summary_file:
        results = csv.DictWriter(
            results_file, fieldnames=["benchmark", "version", "time_ms"]
        )
        summary = csv.DictWriter(
            summary_file,
            fieldnames=["benchmark", "n1", "n2", "change", "significant95", "p"]
        )
        results.writeheader()
        summary.writeheader()
        rng = np.random.default_rng(int(args.seed))
        benefit = get_benefit(int(args.n))
        if (os.path.isdir(a)):
            assert os.path.isdir(b), (
                "If the first argument is a directory, the second must also be a "
                "directory."
            )
            main_dirs(a, b, rng, results, summary, benefit)
        else:
            assert os.path.isfile(a) and os.path.isfile(b), (
                "If the first argument is a file, the second must also be a file."
            )
            main_files(a, b, rng, results, summary, benefit)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Compare the execution times of existing executable."
    )
    parser.add_argument("seed", help="The random seed.")
    parser.add_argument(
        "a", help="The 'main' executable or directory containing executables."
    )
    parser.add_argument(
        "b", help="The 'feature' executable or directory containing executables."
    )
    parser.add_argument(
        "--n", help="A desired number of samples per benchmark. This is taken merely"
        " as a suggestion.", default=20
    )
    main(parser.parse_args())
