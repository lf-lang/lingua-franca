# This script compiles and executes experiments and stores the output of
# those processes to a log file and an output file.
#
# The basic execution model is that in order to run a benchmark with different
# parameters you always run to the same sequence of phases like for example
# compiling and execution. The model assumes that all is controlled via a
# command line with text based commands an parameters.
# Each phase is implemented with a class derived from the class Phase and
# the parameters for each phase are given as parameters for the methods
# involved.
#
# In order to run a benchmark just define a new python file that defines
# a list of phases and a list of lists of commands and command line parameters
# and call runBenchmark() with those as argument. See the files for the
# individual experiments for examples.
#
# The output file is supposed to contain all data of the benchmark and
# additional data to make it reproducible.


import subprocess
import shutil
import os
import datetime
import platform
import importlib
import sys
import argparse


# class Experiment:
#     
#     def __init__(self, outputPath, experimentConfig):
        

def write_header_data(sourcepath, title, gnuplot_file, factor):
    print("set terminal postscript eps enhanced color", file=gnuplot_file)
    print("set output \"%s\"" % (sourcepath.replace(".txt", ".eps")), file=gnuplot_file)
    print("set xlabel \"Cores\"", file=gnuplot_file)
    print("set ylabel \"Execution Time (Milliseconds, Median)\"", file=gnuplot_file)
    
    #if factor < 1:
    #  print("set xtics %d" % (int(4*factor)), file=gnuplot_file)
    #else:
    #print("set xtics 2", file=gnuplot_file)
    
    #print("set xrange [4:]", file=gnuplot_file)
    
    print("set datafile separator \",\"", file=gnuplot_file)
    print("set title \"%s\"" % (title), file=gnuplot_file)
    print("set key outside", file=gnuplot_file)


def doPlot(config, outputPath, benchmarksToRun, implementationsToRun, resultFiles):
    
    parserNames = config.parsers
    parser = {}
    for implName in parserNames:
        parser[implName] = importlib.import_module(parserNames[implName])
    summarizerNames = config.summarizers
    summarizers = {}
    for implName in summarizerNames:
        summarizers[implName] = importlib.import_module(summarizerNames[implName])
    
    plotValues = {}
    plotFilePaths = {}
    
    for implementationName in implementationsToRun:
        plotValues[implementationName] = {}
        resultFilesImpl = resultFiles[implementationName]
        for param, resultFileName in resultFilesImpl.items():
            with open(resultFileName, 'r') as resultFile:
                measurementValues = parser[implementationName].parse(resultFile)
                singleValue = summarizers[implementationName].summarizeMeasurements(measurementValues)
                plotValues[implementationName][param] = singleValue
        
        plotFilePaths[implementationName] = os.path.join(outputPath, implementationName+".plot")
        # write plot data to file to be read by gnuplot
        with open(plotFilePaths[implementationName], 'w') as plotFile:
            values = plotValues[implementationName]
            for param in values.keys():
                plotFile.write(str(param)+","+str(values[param])+'\n')
            plotFile.flush()
    
    gnuPlotFilePath = os.path.join(outputPath, "gnuplot.txt")
    with open(gnuPlotFilePath, 'w') as gnuPlotFile:
        write_header_data("testPath", "myTitle", gnuPlotFile, 1)
        plot_commands = []
        
        for implementationName in implementationsToRun:
            plot_commands.append(f"\"{os.path.basename(plotFilePaths[implementationName])}\" with linespoints")
        
        print("plot " + ",\\\n".join(plot_commands), file=gnuPlotFile)
    subprocess.run(args=['gnuplot', "gnuplot.txt"], cwd=outputPath)


def executePhase(args, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL):
    result = subprocess.run(
            args=args,
            stdin=subprocess.DEVNULL,
            stdout=stdout,
            stderr=stderr)


def main():
    
    parser = argparse.ArgumentParser()
    parser.add_argument('-o', '--output', dest='output', action='store', help='name of the folder where all output is written to; default: "output"')
    parser.add_argument('-c', '--config', dest='config', action='store', help='specify a config file that describes experiments (without file extension .py); default: default_config')
    parser.add_argument('-p', '--skip-plot', dest='skipPlot', action='store_true', help='disable creating plot for experiments; default: false')
    parser.add_argument('-e', '--experiment', dest='experiments', action='append', help='specify one or more experiments to execute by name; default: all experiments in the config file')
    parser.add_argument('-s', '--sequence', dest='sequences', action='append', help='specify one or more sequences within the experiments; default: all sequences in the config file')
    parser.add_argument('-g', '--skip-plot-global', dest='skipPlotGlobal', action='store_true', help='disable creating the overview plot for all experiments; default: false')
    args = parser.parse_args()
    
    # set output folder
    outputPathBase = 'output'
    if args.output:
        outputPathBase = args.output
    
    # load and sanitize specification of config file
    config = None
    configFileName = None
    if args.config:
        configFileName = args.config
    else:
        configFileName = "default_config"
    try:
        config = importlib.import_module(configFileName)
    except:
        print(f'Could not find config file {configFileName}.', file=sys.stderr)
        sys.exit(-1)
    
    # sanitize list of experiments to run
    experimentsToRun = []
    if not args.experiments:
        # run all experiments in config file
        experimentsToRun = config.experiments.keys()
    else:
        for expName in args.experiments:
            if expName in config.experiments.keys():
                experimentsToRun.append(expName)
            else:
                print(f'Experiment "{expName}" not found. Skipping...', file=sys.stderr)
    
    # sanitize list of sequences
    sequencesToRun = []
    if not args.sequences:
        for exp in experimentsToRun:
            for param in config.experiments[exp]:
                for seq in config.experiments[exp][param]:
                    if not seq in sequencesToRun:
                        sequencesToRun.append(seq)
    else:
        # do not sanitize, ignore later if sequence not specified
        sequencesToRun = args.sequences
    #print(sequencesToRun)
    
    timestampString = datetime.datetime.utcnow().strftime('%Y-%m-%d-%H-%M-%S')
    resultFiles = {}
    for implementationName in sequencesToRun:
        resultFiles[implementationName] = {}
    
    for experimentName in experimentsToRun:
        if not experimentName in config.experiments.keys():
            print('Could not find benchmark '+bmNam+'. Skipping...')
        else:
            benchmark = config.experiments[experimentName]
            #TODO sort benchmark.items() before?
            idx = 0 # used for naming the output files
            for paramValue, implementations in benchmark.items():
                
                bmPathBase = os.path.join('output', timestampString, experimentName)
                outputPath = os.path.join(bmPathBase, str(idx))
                os.makedirs(outputPath, exist_ok=True)
                
                for implementationName in sequencesToRun:
                    if not implementationName in implementations.keys():
                        print('Could not find implementation '+implementationName+'. Skipping...')
                    else:
                        implementation = implementations[implementationName]
                        #print('Executing '+experimentName+' implementation '+implementationName)
                        for i in range(len(implementation)):
                            
                            fileName = implementationName+'.result'
                            filePath = os.path.join(outputPath, fileName)
                            resultFiles[implementationName][paramValue] = filePath
                            with open(filePath, 'w') as outputFile:
                                
                                if i < len(implementation)-1:
                                    # not the last phase, simple output log
                                    executePhase(implementation[i], sys.stdout)
                                else:
                                    # last phase, log output to file
                                    executePhase(implementation[i], outputFile, outputFile)
                idx += 1
    
        if not args.skipPlot:
            doPlot(config, bmPathBase, experimentsToRun, sequencesToRun, resultFiles)


if __name__ == "__main__":
    main()
