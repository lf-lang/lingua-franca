set xdata time                           # Indicate that x-axis values are time values
set timefmt "%s"                   # Indicate the pattern the time values will be in
set format x "%s"                     # Set how the dates will be displayed on the plot
 
set xrange ["0":"3"]                    # Set x-axis range of values
set yrange [-.5:.5]                        # Set y-axis range of values
 
set key off                              # Turn off graph legend
set xtics rotate by -45                  # Rotate dates on x-axis 45deg for cleaner display
set title 'Pendulum'            # Set graph title
 
set terminal jpeg                        # Set the output format to jpeg
set output 'output.jpg'                  # Set output file to output.jpg
 
plot 'pendulum.txt' using 1:2 with linespoints linetype 6 linewidth 2