# Trace sequence diagram visualizer

`fedsd` is a utility that reports the interactions (exchanged messages)
between federates and the RTI in a sequence-diagram-like format.

To use `fedsd`, you need to first obtain an execution trace. To do this, enable the tracing mechanism in your Lingua Franca program by setting the `tracing` target property to `true` and then compile and run the program.

This utility starts by transforming each `.lft` file into a `.csv` file, by
internally running `trace_to_csv`. It then aggregates the data from all `.csv`
files to do the matching and draw the sequence diagram.

# Installing

To build `fedsd`, change directory to `./util/tracing` relative to the root of the `lingua-franca` repository  and run `make install`.

# Running

In case the federation is launched using the `bash` script under `bin`, an `.lft` trace
file will be generated for each of the federates, in addition to `rti.lft`. The latter
contains the RTI trace.

If, however, the federation is launched manually, then running the `RTI` command should be passed the `-t` flag in order to make sure that it, too, has tracing enabled:
```
$ RTI -n <number_of_federates> -t
```

It is most convenient to launch the RTI and all federates from the same working directory so that they will all write their trace file to that directory.

Once the federation stopped executing, run `fedsd` on all generated `.lft` files:
```
$ fedsd *.lft
```

The output is an html file named `trace_svg.html` (in the current directory) that contains the sequence of interactions
between the federates and the RTI.
