import os
import sys
import subprocess

def _writePlotHeaderData(gnuplotFile, outputFileName, config, additionalCommands=None):
    # For details see the gnuplot manual.
    
    # set the type of output with 'set terminal <type>' and additional parameters
    print("set terminal pdfcairo enhanced color", file=gnuplotFile)
    
    # default is to output to stdout, change that to file output here
    print(f'set output "{outputFileName}"', file=gnuplotFile)
    
    # labels of the axis
    print(f'set xlabel "{config["plotXAxisLabel"]}"', file=gnuplotFile)
    print(f'set ylabel "{config["plotYAxisLabel"]}"', file=gnuplotFile)
    
    #if factor < 1:
    #  print("set xtics %d" % (int(4*factor)), file=gnuplotFile)
    #else:
    #print("set xtics 2", file=gnuplotFile)
    
    #print("set xrange [4:]", file=gnuplotFile)
    
    # set the separator token in the data files
    print('set datafile separator ","', file=gnuplotFile)
    
    # set the title of the plot
    print(f'set title "{config["plotTitle"]}"', file=gnuplotFile)
    
    # enable showing a legend at position 'outside', the names of the key are
    # specified when plotting
    #print("set key outside", file=gnuplotFile)
    
    if additionalCommands:
        print(additionalCommands, file=gnuplotFile)

def plot(valuesOfSequences, outputPath, experimentId, config):
    
    # initializations
    gnuplotFileName = experimentId + '.txt'
    sequenceDataPaths = {}
    
    # Create a simple CSV files for each sequence that can be referenced in gnuplot. 
    for sequenceId in valuesOfSequences.keys():
        sequenceDataFileName = sequenceId + '.seq'
        sequenceDataPaths[sequenceId] = os.path.join(outputPath, sequenceDataFileName)
        
        with open(sequenceDataPaths[sequenceId], 'w') as sequenceFile:
            sequence = valuesOfSequences[sequenceId]
            for parameter, value in sequence:
                sequenceFile.write(str(parameter) + ',' + str(value) + '\n')
    
    # create file for gnuplot and execute gnuplot
    gnuPlotFilePath = os.path.join(outputPath, gnuplotFileName)
    with open(gnuPlotFilePath, 'w') as gnuPlotFile:
        _writePlotHeaderData(
            gnuplotFile = gnuPlotFile,
            outputFileName = gnuplotFileName.replace('.txt', '.pdf'),
            config = config,
            additionalCommands = config['plotAdditionalGnuplotHeaderCommands'] )
        plot_commands = []
        for sequenceId in valuesOfSequences.keys():
            plot_commands.append(f'"{os.path.basename(sequenceDataPaths[sequenceId])}" title "{config["plotSequenceNames"][sequenceId]}" with linespoints')
            if ('plotSequenceColors' in config) and (sequenceId in config['plotSequenceColors']):
                plot_commands[-1] += f' linecolor "{config["plotSequenceColors"][sequenceId]}"'
        
        print("plot " + ",\\\n".join(plot_commands), file=gnuPlotFile)
    try:
        subprocess.run(args=['gnuplot', gnuplotFileName], cwd=outputPath)
    except Exception as e:
        print('Error: Failed to run "gnuplot".', file=sys.stderr)
        print(e, file=sys.stderr)



