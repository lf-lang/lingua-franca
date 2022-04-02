 
set xrange ["0":"3"]                    # Set x-axis range of values
set yrange [-2:2.5]                        # Set y-axis range of values
 
# set key off                              # Turn off graph legend
set title 'Pendulum'            # Set graph title
 
set terminal png                        # Set the output format to jpeg
set output 'pendulum.png'                  # Set output file to output.jpg

plot 'pendulum.data' using 1:2 with lines linetype 1 linewidth 2 title 'control', \
     'energy.data' using 1:2 with lines linetype 2 linewidth 2 title 'energy', \
     'mode.data' using 1:2 with lines linetype 3 linewidth 2 title 'mode'