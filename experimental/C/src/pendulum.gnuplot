# Gnuplot commands for the FurutaPendulumWithOutput.lf program.

set title 'Pendulum'            # Set graph title

set xrange ["0":"3"]            # Set x-axis range of values
set yrange [-2:2.5]             # Set y-axis range of values

set xlabel "Time (seconds)"
 
set terminal pdf size 5, 3.5    # Set the output format to PDF
set output 'pendulum.pdf'       # Set output file.

set label "SwingUp" at 0.2, -1.2
set label "Catch" at 1.1, -0.2
set label "Stabilize" at 1.2, 1.1

plot 'pendulum.data' using 1:2 with lines linetype 1 linewidth 2 title 'control', \
     'energy.data' using 1:2 with lines linetype 2 linewidth 2 title 'energy', \
     'mode.data' using 1:2 with lines linetype 3 linewidth 2 title 'mode'