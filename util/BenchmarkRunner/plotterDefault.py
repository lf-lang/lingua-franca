import os
import sys
import subprocess

separator = '|'

def _writePlotHeaderData(gnuplotFile, outputFileName, config):
    # For details see the gnuplot manual.
    
    # set the type of output with 'set terminal <type>' and additional parameters
    print("set terminal pdfcairo enhanced color", file=gnuplotFile)
    
    # default is to output to stdout, change that to file output here
    print(f'set output "{outputFileName}"', file=gnuplotFile)
    
    # labels of the axis
    if 'plotXAxisLabel' in config:
        print(f'set xlabel "{config["plotXAxisLabel"]}"', file=gnuplotFile)
    if 'plotYAxisLabel' in config:
        print(f'set ylabel "{config["plotYAxisLabel"]}"', file=gnuplotFile)
    
    #if factor < 1:
    #  print("set xtics %d" % (int(4*factor)), file=gnuplotFile)
    #else:
    #print("set xtics 2", file=gnuplotFile)
    
    #print("set xrange [4:]", file=gnuplotFile)
    
    # set the separator token in the data files
    print(f'set datafile separator "{separator}"', file=gnuplotFile)
    
    # set the title of the plot
    if 'plotTitle' in config:
        print(f'set title "{config["plotTitle"]}"', file=gnuplotFile)
    
    # enable showing a legend at position 'outside', the names of the key are
    # specified when plotting
    print("set key inside left top", file=gnuplotFile)
    
    #if additionalCommands:
    #    print(additionalCommands, file=gnuplotFile)

def plot(valuesOfSequences, outputPath, experimentId, config):
    ''' Plots a simple line plot with gnuplot.
    
    The values are assumed to be tuples in the form (median, firstQuantile, thirdQuantile)
    
    Parameters:
    valuesOfSequences (Dict): The summarized values that the x and y values to plot are taken from.
    outputPath (Str): Where to create the gnuplot file
    experimentId (Str): Unique identifier of the experiment
    config: Configuration of the experiment to load parameter of the plot from.
    '''
    
    # initializations
    gnuplotFileName = experimentId + '.txt'
    sequenceDataPaths = {}
    
    # colors
    colors = None
    if 'plotSequenceColors' in config:
        colors = config['plotSequenceColors']
    
    # Create a simple CSV files for each sequence that can be referenced in gnuplot.
    for sequenceId in valuesOfSequences.keys():
        sequenceDataFileName = sequenceId + '.seq'
        sequenceDataPaths[sequenceId] = os.path.join(outputPath, sequenceDataFileName)
        
        with open(sequenceDataPaths[sequenceId], 'w') as sequenceFile:
            sequence = valuesOfSequences[sequenceId]
            for parameter, value in sequence:
                if value:
                    sequenceFile.write(str(parameter) + separator + str(value[0]) + separator + str(value[1]) + separator + str(value[2]))
                    if colors:
                        sequenceFile.write(separator + colors[sequenceId])
                    sequenceFile.write('\n')
    
    # create file for gnuplot and execute gnuplot
    gnuPlotFilePath = os.path.join(outputPath, gnuplotFileName)
    with open(gnuPlotFilePath, 'w') as gnuPlotFile:
        
        # write common header of gnuplot file
        _writePlotHeaderData(
            gnuplotFile = gnuPlotFile,
            outputFileName = gnuplotFileName.replace('.txt', '.pdf'),
            config = config)
        
        # write one or multiple plot commands
        plot_commands = []
        for sequenceId in valuesOfSequences.keys():
            command = f'"{os.path.basename(sequenceDataPaths[sequenceId])}" using 1:2'
            if colors:
                command += f':5'
            if 'plotSequenceNames' in config:
                command += f' title "{config["plotSequenceNames"][sequenceId]}"'
            command += f' with linespoints'
            if colors:
                command += f' linecolor  variable'
            plot_commands.append(command)
            
            # optional: add yerrorboars here if wanted.
            #command = f'"" using 1:2:3:4 with yerrorbars with linecolor 0'
        
        print("plot " + ",\\\n".join(plot_commands), file=gnuPlotFile)
        
    try:
        subprocess.run(args=['gnuplot', gnuplotFileName], cwd=outputPath)
    except Exception as e:
        print('Error: Failed to run gnuplot.', file=sys.stderr)
        print(e, file=sys.stderr)



