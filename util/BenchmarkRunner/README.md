# Benchmark Runner

This benchmark runner can run predefined sets of benchmarks.
The idea is that it runs multiple different versions of a
benchmark for comparison and plots the results as a visualization.

A set of benchmarks that results in a plot is called an
experiment. A config file can define one or multiple
of these experiments. Each experiment consists of one
or more sequences. Each sequence corresponds roughly
to one line in a simple line plot for comparison.
For each sequence there are one or multiple parameter
values that are used as runtime parameters for running
the benchmarks. Each parameter corresponds to a value
on the x-axis in a simple lineplot.

An example:
One experiment can be the Ping Pong benchmark from the Savina
benchmark suite. Ping Pong is implemented in as two sequences
that each correspond to a implementation and runtime. One
sequence is Akka and one sequence the Lingua Franca implementation
with C++ as the target language. The parameter values of these
two sequences could be a different number of ping messages.
Then the configuration could run the two implementations of
the benchmark for multiple different parameter values, record
the measurements and create a plot with the results.

The benchmark runner is modular and can be extended with new
modules. The three kinds of modules are parsers, summarizers
and plotter. A parser parses the output from running the
benchmarks and compiles a list the measurements. A summarizer
statistically analyzes such a list of measurements and calculates
summarized values like the median for example. The plotter creates
plots from the summarized values.
To implement new modules use the existing ones as examples.

## Setup

The benchmark runner and predefined experiments in the
config files need to be adapted to the environment
before running.

Check requirements in the config file to run.
Set the interface variables inside the config
files according to you setup. Currently you
cannot change these values through any other
interface.


## How to Run

```
$ python3 runner.py --help
```
Shows command line options that are available.

```
$ python3 runner.py
```
Run all experiments from the default configuration file.

```
$ python3 runner.py --config configSavinaMicro
```
Run all experiments from the config file 'configSavinaMicro.py'

```
$ python3 runner.py --config configSavinaMicro --list
```
List all experiments and sequences defined in the given config file.

```
$ python3 runner.py --config configSavinaMicro --experiment PingPong
```
Run only the experiment 'PingPong' from the given config file.

```
$ python3 runner.py --config configSavinaMicro --experiment PingPong --sequence savina-akka-default
```
Run only the Akka implementation of the PingPong experiment.

There are also options to only plot existing measurements,
and change the used summarizer and plotter.

## Issues

This benchmark runner is not very robust in some situations.
Known issues are listed here.

- If the program does not finish due to errors or is killed (CTRL-C) before finishing, the cleanup operations of the benchmarks are not executed.


