# Trace sequence diagram visualizer

`fedsd` is a utility tool that reports the interactions (exchanged messages)
between federates and the RTI in a sequence-diagram-like format.
To enable `fedsd`, `tracing` target property should be set to `true`.

This utility starts by transforming each `.lft` file into a `.csv` file, by
internally running `trace_to_csv`. It then aggregates the data from all `.csv`
files to do the matching and draw the sequence diagram.
# Installing

`fedsd` is installed by calling `make install` under `lingua_franca/util/tracing`.


# Running

In case the federation is launched using the `bash` script under `bin`, an `.lft` trace
file will be generated for each of the federates, in addition to `rti.lft`. This latter
contains the RTI trace.
If, however, the federation is launched manually, then running the `RTI` should
enable tracing, by adding the option `-t`:
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

