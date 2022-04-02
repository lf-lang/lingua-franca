#!/bin/bash
# Build the generated code.
cd ${LF_SOURCE_GEN_DIRECTORY}
cmake .
make

# Move the executable to the bin directory.
mv $1 ${LF_BIN_DIRECTORY}

# Invoke the executable.
${LF_BIN_DIRECTORY}/$1

# Plot the results, which have appeared in the src-gen directory.
gnuplot ${LF_SOURCE_DIRECTORY}/pendulum.gnuplot
open pendulum.png
