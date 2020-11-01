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
import textwrap

# global variables
verbose = False

class Experiment:
    
    # class variables
    _outputFileExtension = '.output'
    
    def _runSequenceForSingleParam(self, sequenceSpec, resultOutput, logOutput):
        
        for commandIdx in range(len(sequenceSpec)):
            shell = False
            if isinstance(sequenceSpec[commandIdx], str):
                shell = True
            if commandIdx < len(sequenceSpec)-1:
                # run command with log output
                try:
                    result = subprocess.run(
                        args=sequenceSpec[commandIdx],
                        shell=shell,
                        stdout=logOutput,
                        stderr=logOutput)
                except:
                    print(f'Error: Failed to run the command: {sequenceSpec[commandIdx].join()}', file=sys.stderr)
            else:
                # run the last command and store output in output file
                try:
                    result = subprocess.run(
                        args=sequenceSpec[commandIdx],
                        shell=shell,
                        stdout=resultOutput,
                        stderr=resultOutput)
                except:
                    print(f'Error: Failed to run the command: {sequenceSpec[commandIdx].join()}', file=sys.stderr)
    
    def _writePlotHeaderData(self, gnuplotFile, outputFileName, additionalCommands=None):
        # For details see the gnuplot manual.
        
        # set the type of output with 'set terminal <type>' and additional parameters
        print("set terminal pdfcairo enhanced color", file=gnuplotFile)
        
        # default is to output to stdout, change that to file output here
        print(f'set output "{outputFileName}"', file=gnuplotFile)
        
        # labels of the axis
        print(f'set xlabel "{self.experimentConfig["plotXAxisLabel"]}"', file=gnuplotFile)
        print(f'set ylabel "{self.experimentConfig["plotYAxisLabel"]}"', file=gnuplotFile)
        
        #if factor < 1:
        #  print("set xtics %d" % (int(4*factor)), file=gnuplotFile)
        #else:
        #print("set xtics 2", file=gnuplotFile)
        
        #print("set xrange [4:]", file=gnuplotFile)
        
        # set the separator token in the data files
        print('set datafile separator ","', file=gnuplotFile)
        
        # set the title of the plot
        print(f'set title "{self.experimentConfig["plotTitle"]}"', file=gnuplotFile)
        
        # enable showing a legend at position 'outside', the names of the key are
        # specified when plotting
        print("set key outside", file=gnuplotFile)
        
        if additionalCommands:
            print(additionalCommands, file=gnuplotFile)
    
    def _processOutputFiles(self, parserNames, summarizerNames, outputFileNames):
        ''' Read, parse, summarize the output files of the experiment given by a parameter.
        Store the complete data for each sequence in the object member self.valuesOfSequences.
        self.valuesOfSequences is a dictionary that stores for each sequence indexed by its name
        a dicitonary that stores all pairs (parameter, value) of the sequence, indexed by the
        parameter value. Example:
        self.valuesOfSequences['seq1'] = { 1: 500, 2: 505, 3: 490 }
        This method also returns self.valuesOfSequences.
        
        Object-level behaviour of method:
        - Assumes valid parameters.
        - Sets the object variable self.valuesOfSequences.
        
        Parameters:
        parserNames (dict): Mapping from sequence identifiers to module names of parsers.
        summarizerNames (dict): Mapping from sequence identifiers to module names of summarizers.
        outputFileNames (list): List of file names of output files generated during running this experiment. This method assumes a valid list.
        '''
        parsers = {}
        for implName in parserNames:
            parsers[implName] = importlib.import_module(parserNames[implName])
        summarizers = {}
        for implName in summarizerNames:
            summarizers[implName] = importlib.import_module(summarizerNames[implName])
        self.valuesOfSequences = {}
        
        for sequenceName in self.sequencesToRun:
            
            # Parse output files of the experiment and summarize the measured values into
            # a single value using the parser and summarizer specified in config file.
            self.valuesOfSequences[sequenceName] = {}
            resultFilesOfSequence = outputFileNames[sequenceName]
            for parameter, resultFileName in resultFilesOfSequence.items():
                with open(resultFileName, 'r') as resultFile:
                    measurementValues = parsers[sequenceName].parse(resultFile, resultFileName)
                    summarizedValue = summarizers[sequenceName].summarizeMeasurements(measurementValues, parameter, resultFileName)
                    self.valuesOfSequences[sequenceName][parameter] = summarizedValue
        
        return self.valuesOfSequences
    
    def _compileListOfExistingOutputFiles(self):
        '''Compiles a list of paths to output files created when running this experiment.
        
        Object-level behaviour of method:
        - Assumes the object-wide naming conventions for folders and files.
        - Uses self.outputPath as the base directory to search for output files.
        - Sets and overwrites the object variable self.outputFileNames.
        '''
        self.outputFileNames = {} # reset object member
        folders = next(os.walk(self.outputPath))[1]
        
        for folder in folders:
            curentPath = os.path.join(self.outputPath, folder)
            files = next(os.walk(curentPath))[2]
            for file in files:
                if file.endswith(Experiment._outputFileExtension):
                    seqNameFromFileName = file[:(-len(Experiment._outputFileExtension))]
                    if seqNameFromFileName in self.sequencesToRun:
                        if not seqNameFromFileName in self.outputFileNames:
                            self.outputFileNames[seqNameFromFileName] = {}
                        self.outputFileNames[seqNameFromFileName][folder] = os.path.join(curentPath, file)
        
        return self.outputFileNames
                    
    
    def __init__(self, experimentId, experimentConfig, outputPathBase, sequencesToRun, sequenceNames):
        '''Create a new Experiment.
        
        Parameters:
        experimentId (str): Uniquely identifies the experiment.
        experimentConfig (dict): Contains config parameters and the commands to run the experiment.
        outputPathBase (str): The folder in which the experiments creates/reads its own private folder corresponding to the experimentId.
        sequencesToRun (list): List of identifiers of all sequences to be run.
        sequenceNames (dict): Maps the identifiers of sequences to human-readable names.
        '''
        
        self.experimentId = experimentId
        self.outputPathBase = outputPathBase
        self.outputPath = os.path.join(outputPathBase, experimentId)
        self.experimentConfig = experimentConfig
        self.sequences = experimentConfig['sequences']
        self.sequencesToRun = sequencesToRun
        self.sequenceNames = sequenceNames
        self.name = experimentConfig['plotTitle']
    
    def getValueForGlobalPlot(self, sequenceName):
        parameter = self.experimentConfig['sequenceParameterForGlobalPlot']
        if sequenceName in self.valuesOfSequences:
            return self.valuesOfSequences[sequenceName][parameter]
        else:
            return None
    
    def getGlobalPlotLabel(self):
        return self.experimentConfig['globalPlotXAxisLabel']
    
    def plotExperiment(self, parserNames, summarizerNames, sequenceColors=None):
        # initializations
        gnuplotFileName = self.experimentId + '.txt'
        sequenceDataPaths = {}
        
        if not hasattr(self, 'outputFileNames'):
            # assuming experiment did not run, searching existing output files
            self._compileListOfExistingOutputFiles()
        self._processOutputFiles(parserNames, summarizerNames, self.outputFileNames)
        
        for sequenceName in self.sequencesToRun:
            
            # Output the pairs (parameter, summarized value) of each sequence to a simple
            # CSV file that can be read by gnuplot.
            sequenceDataFileName = sequenceName + '.seq'
            sequenceDataPaths[sequenceName] = os.path.join(self.outputPath, sequenceDataFileName)
            
            with open(sequenceDataPaths[sequenceName], 'w') as sequenceFile:
                sequence = self.valuesOfSequences[sequenceName]
                for parameter, value in sequence.items():
                    sequenceFile.write(str(parameter) + ',' + str(value) + '\n')
        
        # create file for gnuplot and execute gnuplot
        gnuPlotFilePath = os.path.join(self.outputPath, gnuplotFileName)
        with open(gnuPlotFilePath, 'w') as gnuPlotFile:
            self._writePlotHeaderData(
                gnuplotFile = gnuPlotFile,
                outputFileName = gnuplotFileName.replace('.txt', '.pdf'),
                additionalCommands = self.experimentConfig['additionalGnuplotHeaderCommands'] )
            plot_commands = []
            for sequenceName in self.sequencesToRun:
                plot_commands.append(f'"{os.path.basename(sequenceDataPaths[sequenceName])}" title "{self.sequenceNames[sequenceName]}" with linespoints')
                if sequenceColors:
                    plot_commands[-1] += f' linecolor {sequenceColors["sequenceName"]}'
            
            print("plot " + ",\\\n".join(plot_commands), file=gnuPlotFile)
        try:
            subprocess.run(args=['gnuplot', gnuplotFileName], cwd=self.outputPath)
        except:
            print('Failed to run "gnuplot".', file=sys.stderr)
    
    
    def runExperiment(self, logOutput=sys.stdout):
        '''
        Object-level behaviour of method:
        - Sets and overwrites the object variable self.outputFileNames.
        - ...
        '''
        
        os.makedirs(self.outputPath, exist_ok=True)
        self.outputFileNames = {}
        for sequenceName in self.sequencesToRun:
            self.outputFileNames[sequenceName] = {}
        
        numParameters = len(self.sequences)
        numRunParameters = 0
        for parameterValue, dictOfSequencesForParam in self.sequences.items():
            numRunParameters += 1
            if(verbose):
                print(f'    Running parameter {numRunParameters}/{numParameters}')
            
            # create output dir
            outputPathForParameter = os.path.join(self.outputPath, parameterValue)
            os.makedirs(outputPathForParameter, exist_ok=True)
            
            # run all sequences for a single parameter
            for sequenceName, sequenceSpec in dictOfSequencesForParam.items():
                # run sequence only if in list
                if sequenceName in self.sequencesToRun:
                    outputFilePath = os.path.join(outputPathForParameter, sequenceName + Experiment._outputFileExtension)
                    self.outputFileNames[sequenceName][parameterValue] = outputFilePath
                    with open(outputFilePath, 'w') as outputFile:
                        self._runSequenceForSingleParam(sequenceSpec, outputFile, logOutput)
            

def writeGlobalHeaderData(gnuplotFile, outputFileName, dataFileName, globalPlotConfig):
    # For details see the gnuplot manual.
    
    # set the type of output with 'set terminal <type>' and additional parameters
    print("set terminal pdfcairo enhanced color", file=gnuplotFile)
    
    # default is to output to stdout, change that to file output here
    print(f'set output "{outputFileName}"', file=gnuplotFile)
    
    # labels of the axis
    #print(f'set xlabel "{self.experimentConfig["plotXAxisLabel"]}"', file=gnuplotFile)
    print(f'set ylabel "{globalPlotConfig["plotYAxisLabel"]}"', file=gnuplotFile)
    
    # set the separator token in the data files
    print('set datafile separator ","', file=gnuplotFile)
    
    # set the title of the plot
    print(f'set title "{globalPlotConfig["plotTitle"]}"', file=gnuplotFile)
    
    # enable showing a legend at position 'outside', the names of the key are
    # specified when plotting
    #print("set key outside", file=gnuplotFile)
    
    # paint histograms
    print(f'set style data histograms', file=gnuplotFile)
    # subtype cluster from histograms
    print(f'set style histogram cluster', file=gnuplotFile)
    print(f'set style fill solid border', file=gnuplotFile)
    print(f'set key autotitle columnhead', file=gnuplotFile)
    print(f'plot for [i=2:*] "{dataFileName}" using i:xtic(1)', file=gnuplotFile)
    
    # user defined commands
    print(globalPlotConfig["additionalGnuplotHeaderCommands"], file=gnuplotFile)


def main():
    
    parser = argparse.ArgumentParser()
    parser.add_argument('-o', '--output', dest='output', action='store', help='name of the folder where all output is written to; default: "output"')
    parser.add_argument('-c', '--config', dest='config', action='store', help='specify a config file that describes experiments (without file extension .py); default: default_config')
    parser.add_argument('-p', '--skip-plotting', dest='skipPlot', action='store_true', help='disable creating plot for experiments; default: false')
    parser.add_argument('-e', '--experiment', dest='experiments', action='append', help='specify one or more experiments to execute by name; default: all experiments in the config file')
    parser.add_argument('-s', '--sequence', dest='sequences', action='append', help='specify one or more sequences within the experiments; default: all sequences in the config file')
    parser.add_argument('-g', '--skip-plotting-global', dest='skipPlotGlobal', action='store_true', help='disable creating the overview plot for all experiments; default: false')
    parser.add_argument('-r', '--plot-only', dest='plotOnly', action='store_true', help='do not run benchmarks, only create plots from a given output directory; for this option to work an output directory using "-o" is needed')
    parser.add_argument('-v', '--verbose', dest='verbose', action='store_true', help='enable a more verbose output')
    parser.add_argument('-l', '--list', dest='list', action='store_true', help='list all experiments and sequence identifiers in the specified config file')
    args = parser.parse_args()
    
    # handle verbose arg
    global verbose
    if args.verbose:
        verbose = True
    
    # load and sanitize specification of config file
    config = None
    configFileName = 'default_config'
    if args.config:
        configFileName = args.config
    config = importlib.import_module(configFileName)
    
    # handle option --list: print and exit
    if args.list:
        print(f'Config file: {configFileName}\n')
        print(f'List of all experiments by identifier:\n')
        for experimentId in config.experiments:
            print(f' {experimentId}')
            prefix = '    '
            preferredWidth = 75
            wrapper = textwrap.TextWrapper(initial_indent=prefix, width=preferredWidth, subsequent_indent=' '*len(prefix))
            print(wrapper.fill(config.experiments[experimentId]['description']))
        print('')
        print('List of all sequences by identifier (pretty name):')
        print(' Note: Not all sequences are available in all experiments.\n')
        listOfSequenceIds = []
        for exp in config.experiments:
            for param in config.experiments[exp]['sequences'].keys():
                for seq in config.experiments[exp]['sequences'][param]:
                    if not seq in listOfSequenceIds:
                        print(f' {seq}: {config.sequenceNames[seq]}')
                        listOfSequenceIds.append(seq)
        sys.exit(0)
    
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
            for param in config.experiments[exp]['sequences'].keys():
                for seq in config.experiments[exp]['sequences'][param]:
                    if not seq in sequencesToRun:
                        sequencesToRun.append(seq)
    else:
        for exp in experimentsToRun:
            for param in config.experiments[exp]['sequences'].keys():
                for seq in config.experiments[exp]['sequences'][param]:
                    if (seq in args.sequences) and (not seq in sequencesToRun):
                        sequencesToRun.append(seq)
    
    # plot-only?
    plotOnly = False
    if args.plotOnly:
        plotOnly = True
        if args.skipPlot and args.skipPlotGlobal:
            print(f'Warning: Plot only specified, but skipping all plot types. Nothing to do.', file=sys.stderr)
            sys.exit(0)
    
    # determine output directory from args
    basePath = None
    if plotOnly:
        if not args.output:
            print(f'Error: Need path with option -o for plot only mode.', file=sys.stderr)
            sys.exit(-1)
        if not os.path.exists(args.output):
            print(f'Error: Could not find {args.output}.', file=sys.stderr)
            sys.exit(-1)
        basePath = args.output
    else:
        outputBasePath = 'output'
        if args.output:
            outputBasePath = args.output
        timestampString = datetime.datetime.now().strftime('%Y-%m-%d_%H%M%S')
        basePath = os.path.join(outputBasePath, timestampString)
        os.makedirs(basePath, exist_ok=True)
    
    # start execution
    logFilePath = os.path.join(basePath, 'log.txt')
    with open(logFilePath, 'w') as logFile:
        
        # create objects representing the experiments
        experiments = []
        for experimentId in experimentsToRun:
            experimentConfig = config.experiments[experimentId]
            experiment = Experiment(
                experimentId = experimentId,
                outputPathBase = basePath,
                experimentConfig = experimentConfig,
                sequencesToRun = sequencesToRun,
                sequenceNames = config.sequenceNames)
            experiments.append(experiment)
        
        # execute experiments
        if not plotOnly:
            # run experiments
            numExperimentsRun = 0
            for experiment in experiments:
                numExperimentsRun += 1
                if verbose:
                    print(f'Running Experiment {numExperimentsRun}/{len(experimentsToRun)}: {experimentId}')
                experiment.runExperiment(logFile)
                
                # plot the experiment
                if not args.skipPlot:
                    experiment.plotExperiment(config.parsers, config.summarizers)
        
        # create global plot
        if not args.skipPlotGlobal:
            # create global overview graph for all experiments executed
            globalPlotFileName = "overview.dat"
            globalPlotFilePath = os.path.join(basePath, globalPlotFileName)
            with open(globalPlotFilePath, 'w') as globalPlotFile:
                
                # write sequence names in first line of the data file
                globalPlotFile.write(',' + ','.join(sequencesToRun) + '\n')
                
                # collect data and write line by line
                for experiment in experiments:
                    items = [experiment.getGlobalPlotLabel()]
                    for sequenceName in sequencesToRun:
                        if experiment.getValueForGlobalPlot(sequenceName):
                            items.append(str(experiment.getValueForGlobalPlot(sequenceName)))
                        else:
                            items.append('0')
                    globalPlotFile.write(','.join(items) + '\n')
            
            gnuplotFileName = 'gnuplot.txt'
            gnuplotFilePath = os.path.join(basePath, gnuplotFileName)
            with open(gnuplotFilePath, 'w') as gnuplotFile:
                writeGlobalHeaderData(
                    gnuplotFile = gnuplotFile,
                    outputFileName = 'overview.pdf',
                    dataFileName = globalPlotFileName,
                    globalPlotConfig = config.globalPlot )
            
            try:
                subprocess.run(args=['gnuplot', gnuplotFileName], cwd=basePath)
            except:
                print('Failed to run "gnuplot".', file=sys.stderr)

if __name__ == "__main__":
    main()
