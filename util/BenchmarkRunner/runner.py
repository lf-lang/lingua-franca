# This script is intended to configure, compile and execute benchmarks and
# generate a plot for the results.
#
# A set of benchmark runs that are plotted together is called an experiment.
# Config files specify one or multiple experiments, they store parameters
# and how to compile and run the benchmarks.
# By default all experiments inside a config file are run, but the selection
# can be restricted with command line parameters to this script.
# Each experiment consists of one or more sequences.
# In a simple line plot of an experiment each sequence corresponds to one line.
#
# New config files can be written and used via command line parameter.
# The runner can also be extended by writing new parsers, summarizers and plotters.
# Look at the existing files for examples.


import subprocess
import shutil
import os
import datetime
import platform
import importlib
import sys
import argparse
import textwrap


class Experiment:
    
    # class variables
    _outputFileExtension = '.output'
    
    def _runSequenceForSingleParam(self, sequenceSpec, resultOutput, logOutput, statusOutput=sys.stdout, errorOutput=sys.stderr, returnOnError=True):
        ''' Run a list of commands one by one and write the outputs of the last command to resultOutput.
        
        Return:
        True if all commands finished successfully, otherwise False
        
        Parameters:
        sequenceSpec (list): List containing strings and lists of strings that are run through a shell or directly. Strings are run through a shell, lists of strings directly.
        resultOutput: Object with a write method that stores the outputs, usually an open file.
        logOutput: Object with a write method that stores log messages, usually an open file.
        '''
        
        for commandIdx in range(len(sequenceSpec)):
            
            # execute a list of string without shell and a simple string through a shell
            # this allows to use shell builtins within the commands
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
                except Exception as e:
                    print(f'Error: Failed to run the command: {sequenceSpec[commandIdx]}', file=errorOutput)
                    print(e, file=errorOutput)
                    return False
            else:
                # run the last command and store output in output file
                try:
                    result = subprocess.run(
                        args=sequenceSpec[commandIdx],
                        shell=shell,
                        stdout=resultOutput,
                        stderr=resultOutput)
                except Exception as e:
                    print(f'Error: Failed to run the command: {sequenceSpec[commandIdx]}', file=errorOutput)
                    print(e, file=errorOutput)
                    return False
        return True
    
    def _processOutputFiles(self, outputFileNames):
        ''' Read, parse, summarize the output files of the experiment given by a parameter.
        Stores the data per sequence in the object member self.valuesOfSequences.
        self.valuesOfSequences is a dictionary that stores for each sequence indexed by its name
        a dicitonary that stores all pairs (parameter, value) of the sequence, indexed by the
        parameter value. Example:
        self.valuesOfSequences['seq1'] = { 1: 500, 2: 505, 3: 490 }
        This method also returns self.valuesOfSequences.
        
        Parameters:
        outputFileNames (dict): Mapping of sequence names to a list of output file names. This method assumes a valid list.
        '''
        # reset object variable for the results
        self.valuesOfSequences = {}
        
        # load parsers
        parsers = {}
        for seqId in self.experimentConfig['parsers'].keys():
            parsers[seqId] = importlib.import_module(self.experimentConfig['parsers'][seqId])
        
        # parse and summarize all output files in the list outputFileNames
        for sequenceId in outputFileNames.keys():
            if sequenceId in self.sequencesToRun:
                self.valuesOfSequences[sequenceId] = []
                resultFilesOfSequence = outputFileNames[sequenceId]
                for parameter, resultFileName in resultFilesOfSequence:
                    with open(resultFileName, 'r') as resultFile:
                        measurementValues = parsers[sequenceId].parse(resultFile, resultFileName)
                        summarizedValue = self.summarizer.summarizeMeasurements(measurementValues, parameter, resultFileName)
                        if summarizedValue:
                            self.valuesOfSequences[sequenceId].append( (parameter, summarizedValue) )
        
        return self.valuesOfSequences
    
    def _compileListOfExistingOutputFiles(self):
        '''Compiles a list of paths to output files created when running this experiment.
        
        Object-level behaviour of method:
        - Assumes the object-wide naming conventions for folders and files.
        - Uses self.outputPath as the base directory to search for output files.
        - Overwrites and sets the object variable self.outputFileNames.
        '''
        self.outputFileNames = {} # reset object member
        
        # collect all folders that are supposed to correspond to parameters
        folders = []
        for dirpath, dirnames, filenames in os.walk(self.outputPath):
            folders.extend(dirnames)
            break
        
        for parameter, seqSpecs in self.sequences:
            # find corresponding folder for param in specification
            folderForCurrentParam = None
            for folder in folders:
                if folder == str(parameter):
                    folderForCurrentParam = folder
                    break
            
            if folderForCurrentParam:
                curentPath = os.path.join(self.outputPath, folderForCurrentParam)
                
                # collect all files in folder corresponding to parameter
                files = []
                for dirpath, dirnames, filenames in os.walk(curentPath):
                    files.extend(filenames)
                    break
                
                for file in files:
                    # check if file is valid file
                    if file.endswith(Experiment._outputFileExtension) and \
                      (os.path.splitext(file)[0] in seqSpecs.keys()) and \
                      (os.path.splitext(file)[0] in self.sequencesToRun):
                        seqNameFromFileName = file[:(-len(Experiment._outputFileExtension))]
                        if not seqNameFromFileName in self.outputFileNames:
                            self.outputFileNames[seqNameFromFileName] = []
                        self.outputFileNames[seqNameFromFileName].append( (parameter, os.path.join(curentPath, file)) )
        
        return self.outputFileNames


    def __init__(self, experimentId, experimentConfig, outputPathBase, sequencesToRun, plotter, summarizer):
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
        self.name = experimentConfig['plotTitle']
        self.plotter = plotter
        self.summarizer = summarizer
    
    def plotExperiment(self):
        ''' Creates the plot for the results of this experiment.
        '''
        
        if not hasattr(self, 'outputFileNames'):
            # assuming experiment did not run, search for existing output files in base directory
            self._compileListOfExistingOutputFiles()
        try:
            self._processOutputFiles(self.outputFileNames)
        except Exception as e:
            print(f'Could not plot experiment {self.experimentId}', file=sys.stderr)
            print(e, file=sys.stderr)
            return
        
        try:
            self.plotter.plot(
                valuesOfSequences = self.valuesOfSequences,
                outputPath = self.outputPath,
                experimentId = self.experimentId,
                config = self.experimentConfig)
        except Exception as e:
            print(f'Could not plot experiment {self.experimentId}', file=sys.stderr)
            print(e, file=sys.stderr)
            return
    
    def runExperiment(self, verbose, logOutput=sys.stdout):
        ''' Runs the commands specified for the experiment and stores the output in output files.
        
        Object-level behaviour of method:
        - Sets and overwrites the object variable self.outputFileNames.
        '''
        
        if 'initializeExperiment' in self.experimentConfig:
            self._runSequenceForSingleParam(sequenceSpec = self.experimentConfig['initializeExperiment'],
                                            resultOutput =  logOutput,
                                            logOutput = logOutput,
                                            returnOnError = False)
        
        os.makedirs(self.outputPath, exist_ok=True)
        self.outputFileNames = {}
        
        numParameters = len(self.sequences)
        numParametersRun = 0
        for parameterValue, dictOfSequencesForParam in self.sequences:
            numParametersRun += 1
            if(verbose):
                print(f'  Running parameter {numParametersRun}/{numParameters}: {parameterValue}')
            
            # create output dir
            outputPathForParameter = os.path.join(self.outputPath, str(parameterValue))
            os.makedirs(outputPathForParameter, exist_ok=True)
            
            # run init commands if available
            if 'initExperiment' in self.experimentConfig:
                self._runSequenceForSingleParam(sequenceSpec = self.experimentConfig['initExperiment'],
                                                resultOutput = logOutput,
                                                logOutput = logOutput,
                                                returnOnError = False)
            
            # run all sequences for a single parameter
            for sequenceName, sequenceSpec in dictOfSequencesForParam.items():
                # run sequence only if in list
                if sequenceName in self.sequencesToRun:
                    
                    if(verbose):
                        print(f'   Sequence {sequenceName}...')
                    outputFilePath = os.path.join(outputPathForParameter, sequenceName + Experiment._outputFileExtension)
                    if not sequenceName in self.outputFileNames:
                        self.outputFileNames[sequenceName] = []
                    self.outputFileNames[sequenceName].append( (parameterValue, outputFilePath) )
                    with open(outputFilePath, 'w') as outputFile:
                        result = self._runSequenceForSingleParam(sequenceSpec, outputFile, logOutput)
                        #TODO do something with result?
                        if not result:
                            print('Warning: Error while executing {sequenceName}. Check log and output file for details.', file=sys.stderr)
            
            # run cleanup commands if available
            if 'cleanupExperiment' in self.experimentConfig:
                result = self._runSequenceForSingleParam(sequenceSpec = self.experimentConfig['cleanupExperiment'],
                                                resultOutput = logOutput,
                                                logOutput = logOutput,
                                                returnOnError = False)
                if not result:
                    print(f'Warning: Error while cleaning up after executing experiment {self.experimentId}.', file=sys.stderr)
        
    def finalCleanupExperiment(self, logOutput=sys.stdout):
        if 'finalCleanupExperiment' in self.experimentConfig:
            result = self._runSequenceForSingleParam(sequenceSpec = self.experimentConfig['finalCleanupExperiment'],
                                            resultOutput = logOutput,
                                            logOutput = logOutput,
                                            returnOnError = False)
            if not result:
                print(f'Warning: Error while final clean up for experiment {self.experimentId}.', file=sys.stderr)
            


def main():
    
    parser = argparse.ArgumentParser()
    parser.add_argument('-o', '--output', dest='output', action='store', help='name of the folder where all output is written to; default: "output"')
    parser.add_argument('-c', '--config', dest='config', action='store', help='specify a config file that describes experiments (without file extension .py); default: configDefault')
    parser.add_argument('-n', '--skip-plotting', dest='skipPlot', action='store_true', help='disable creating plot for experiments; default: false')
    parser.add_argument('-e', '--experiment', dest='experiments', action='append', help='specify one or more experiments to execute by name; default: all experiments in the config file')
    parser.add_argument('-s', '--sequence', dest='sequences', action='append', help='specify one or more sequences within the experiments; default: all sequences in the config file')
    parser.add_argument('-m', '--summarizer', dest='summarizer', action='store', help='specify the summarizer for statistical analysis of the measurements; default: defaultSummarizer')
    parser.add_argument('-p', '--plotter', dest='plotter', action='store', help='specify which plotter creates the plots; default: defaultPlotter')
    parser.add_argument('-r', '--plot-only', dest='plotOnly', action='store_true', help='do not run benchmarks, only create plots from a given output directory; for this option to work an output directory using "-o" is needed')
    parser.add_argument('-q', '--quiet', dest='quiet', action='store_true', help='disable progress output to stdout')
    parser.add_argument('-l', '--list', dest='list', action='store_true', help='list all experiments and sequence identifiers in the specified config file')
    args = parser.parse_args()
    
    # handle verbose arg
    verbose = True
    if args.quiet:
        verbose = False
    
    # plot-only?
    plotOnly = False
    if args.plotOnly:
        plotOnly = True
        if args.skipPlot:
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
    basePath = os.path.abspath(basePath)
    
    # change working dir to directory containing this file
    abspath = os.path.abspath(__file__)
    dname = os.path.dirname(abspath)
    os.chdir(dname)
    
    # load and sanitize specification of config file
    config = None
    configFileName = 'configSavinaMicro'
    if args.config:
        configFileName = args.config
        if configFileName.endswith('.py'):
            configFileName = configFileName.removesuffix('.py')
    try:
        config = importlib.import_module(configFileName)
    except Exception as e:
        print(f'Could not load configuration {configFileName}', file=sys.stderr)
        print(e, file=sys.stderr)
        sys.exit(-1)
    
    # handle summarizer
    summarizer = None
    summarizerFileName = 'summarizerDefault'
    if args.summarizer:
        summarizerFileName = args.summarizer
        if summarizerFileName.endswith('.py'):
            summarizerFileName = summarizerFileName.removesuffix('.py')
    try:
        summarizer = importlib.import_module(summarizerFileName)
    except Exception as e:
        print(f'Could not load summarizer {summarizerFileName}', file=sys.stderr)
        print(e, file=sys.stderr)
        sys.exit(-1)
    
    # handle plotter
    plotter = None
    plotterFileName = 'plotterDefault'
    if args.plotter:
        plotterFileName = args.plotter
        if plotterFileName.endswith('.py'):
            plotterFileName = plotterFileName.removesuffix('.py')
    try:
        plotter = importlib.import_module(plotterFileName)
    except Exception as e:
        print(f'Could not load plotter {plotterFileName}', file=sys.stderr)
        print(e, file=sys.stderr)
        sys.exit(-1)
    
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
            for param, sequences in config.experiments[exp]['sequences']:
                for seq in sequences:
                    if not seq in sequencesToRun:
                        sequencesToRun.append(seq)
    else:
        for exp in experimentsToRun:
            for param, sequences in config.experiments[exp]['sequences']:
                for seq in sequences:
                    if (seq in args.sequences) and (not seq in sequencesToRun):
                        sequencesToRun.append(seq)
    
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
                plotter = plotter,
                summarizer = summarizer)
            experiments.append(experiment)
        
        # handle each experiment
        numExperimentsRun = 0
        for experiment in experiments:
            
            # run the experiment
            if not plotOnly:
                numExperimentsRun += 1
                if verbose:
                    print(f'Running experiment {numExperimentsRun}/{len(experimentsToRun)}: {experiment.experimentId}')
                experiment.runExperiment(verbose, logFile)
            
            # plot the experiment
            if not args.skipPlot:
                experiment.plotExperiment()
        
        # cleaup experimentes
        if not plotOnly:
            for experiment in experiments:
                experiment.finalCleanupExperiment()



if __name__ == "__main__":
    main()
