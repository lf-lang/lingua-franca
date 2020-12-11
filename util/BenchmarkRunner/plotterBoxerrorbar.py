''' This plotter expects 3-tuples
(median, firstQuantile, thirdQuantile) as input.
The output is the default Boxerrorbar plot
from gnuplot.
'''

import os
import sys
import subprocess

separator = '|'

def _writePlotHeaderData(gnuplotFile, outputFileName, config, additionalCommands=None):
    
    # For details see the gnuplot manual.
    # See https://stackoverflow.com/questions/62848395/horizontal-bar-chart-in-gnuplot
    
    print("set terminal pdfcairo enhanced color size 7cm, 7cm", file=gnuplotFile)
    print(f'set output "{outputFileName}"', file=gnuplotFile)
    print(f'set datafile separator "{separator}"', file=gnuplotFile)
    
    print('set style fill solid', file=gnuplotFile)
    print('unset key', file=gnuplotFile)
    #print('set pointsize 0', file=gnuplotFile)
    #print(f'set xlabel "{config["plotYAxisLabel"]}"', file=gnuplotFile)
    print('set boxwidth 0.4 relative', file=gnuplotFile)
    print(f'set title "{config["plotTitle"]}"', file=gnuplotFile)
    
    #print('myBoxWidth = 0.8', file=gnuplotFile)
    #print('set offsets 0,0,0.5-myBoxWidth/2.,0.5', file=gnuplotFile)
    
    if additionalCommands:
        print(additionalCommands, file=gnuplotFile)

def plot(valuesOfSequences, outputPath, experimentId, config):
    
    # initializations
    gnuplotFileName = experimentId + '.txt'
    dataFileName = f'{experimentId}.dat'
    colors = config.get('plotSequenceColors', {})
    
    # Create simple CSV file that can be referenced in gnuplot.
    with open(os.path.join(outputPath, dataFileName), 'w') as dataFile:
        
        # find the correct order of the sequences to plot
        orderSequences = None
        if 'plotArrangementSequences' in config:
            orderSequences = config.get('plotArrangementSequences')
        else:
            orderSequences = valuesOfSequences.keys()
        
        # write data according to the order
        for sequenceId in orderSequences:
            if sequenceId in valuesOfSequences.keys():
                sequence = valuesOfSequences[sequenceId]
                parameter, value = sequence[0]
                #dataFile.write(config['plotSequenceNames'][sequenceId] + separator + str(value[0]) + separator +
                #               str(value[1]) + separator + str(value[2]) + separator + str(colors.get(sequenceId, 1)) + '\n')
                dataFile.write(config['plotSequenceNames'][sequenceId] + separator + str(value[0]) + separator +
                               str(value[1]) + separator + str(value[2]) + separator + str(colors.get(sequenceId, 1)) + '\n')
    
    # create file for gnuplot and execute gnuplot
    gnuPlotFilePath = os.path.join(outputPath, gnuplotFileName)
    with open(gnuPlotFilePath, 'w') as gnuPlotFile:
        _writePlotHeaderData(
            gnuplotFile = gnuPlotFile,
            outputFileName = gnuplotFileName.replace('.txt', '.pdf'),
            config = config,
            additionalCommands = config.get('plotAdditionalGnuplotHeaderCommands') )
        plot_commands = []
        #plot_commands.append(f'"{dataFileName}" using ($0):2:3:4:xticlabels(1) with boxes linecolor variable')
        #plot_commands.append(f'"{dataFileName}" using ($0):2:5:xticlabels(1) with boxes linecolor variable, "" using ($0):2:3:4 with yerrorbars linecolor 0')
        plot_commands.append(f'"{dataFileName}" using ($0):2:5:xticlabels(1) with boxes linecolor variable, "" using ($0):2:3:4 with yerrorbars linecolor 0')
         
        print("plot " + ",\\\n".join(plot_commands), file=gnuPlotFile)
    try:
        subprocess.run(args=['gnuplot', gnuplotFileName], cwd=outputPath)
    except Exception as e:
        print('Warning: Failed to run "gnuplot", run it manually.', file=sys.stderr)
        print(e, file=sys.stderr)



