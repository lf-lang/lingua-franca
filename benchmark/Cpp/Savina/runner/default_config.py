# The config file specifies how to run benchmarks and plot the results.
# First, you need to provide parsers (for parsing the output of benchmarks)
# and summarizers (for summarizing the measurements into a single value for plotting)
# as python modules or reuse the predefined parsers and summarizers.
# Second, you need to write a config file like this that provides the required
# variables shown below. Look at this example config file to write your
# own config files.
#
# The config files is written directly in python an dynamically loaded
# by the runner (controlled by command line arguments). The advantage of
# writing the config directly in python is that you can use the full power
# of it to write the specifications that often are very repetitive. That
# also makes it easy to combine multiple config files into one or structure
# your config in other ways.
# Which experiments are actually run is determined not by the config files
# but by command line arguments. The default is to run all experiments
# specified in the config file.
#
# Required global variables are:
# parsers (dict): mapping from sequence identifiers to parser modules in python
# summarizers (dict): mapping from sequence identifiers to summarizer modules in python
# globalPlot (dict): mapping from pre-defined attribute identifiers to values used for plotting of the overview plot
# sequenceNames (dict): mapping from the identifiers of sequences to human-readable names, used for plotting
# experiments (dict): specifications of experiments, definition of how to run sequences and additional predefined attributes, mostly for plotting
#
# Optional variables:
# sequenceColors (dict): mapping from sequence identifiers to colors for plotting

import os.path

parsers = {
    'savina-akka-default': 'savina_parser',
    'lf1': 'lf_parser',
    'lf2': 'lf_parser',
    'lf4': 'lf_parser',
    'lf8': 'lf_parser',
    'lf16': 'lf_parser',
    'lf-c-1': 'lf_c_parser'
}

summarizers = {
    'savina-akka-default': 'summarizer_median_warmup',
    'lf1': 'summarizer_median_warmup',
    'lf2': 'summarizer_median_warmup',
    'lf4': 'summarizer_median_warmup',
    'lf8': 'summarizer_median_warmup',
    'lf16': 'summarizer_median_warmup',
    'lf-c-1': 'summarizer_median_warmup'
}

# optional colors for gnuplot
# use 'show colornames' in gnuplot to get a list.
# examples: white black dark-grey red web-green web-blue dark-magenta dark-cyan dark-orange
# dark-yellow royalblue goldenrod dark-spring-green purple steelblue dark-red dark-chartreuse orchid
# aquamarine brown yellow turquoise grey light-red light-green light-blue...
sequenceColors = {
    'savina-akka-default': 'violet',
    'lf1': 'dark-turquoise',
    'lf2': 'skyblue',
    'lf4': 'brown',
    'lf8': 'turquoise',
    'lf16': 'dark-spring-green',
    'lf-c-1': 'web-blue'
}

# specs for global overview plot
globalPlot = {
    'plotTitle': 'Overview for all benchmarks',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'additionalGnuplotHeaderCommands': ''
}

# names for the sequences
sequenceNames = {
    'lf1': 'Lingua Franca Cpp (1 thread)',
    'lf2': 'Lingua Franca Cpp (2 threads)',
    'lf4': 'Lingua Franca Cpp (4 threads)',
    'lf8': 'Lingua Franca Cpp (8 threads)',
    'lf16': 'Lingua Franca Cpp (16 threads)',
    'savina-akka-default': 'Savina default config',
    'lf-c-1': 'Lingua Franca C (1 thread)'
}

# helper and convenience variables
lfSourceFilePathBase = '..'
savinaJarPath = '../../../../../../savina/target/savina-0.0.1-SNAPSHOT-jar-with-dependencies.jar'
savinaPackagePathBase = 'edu.rice.habanero.benchmarks'
externalLfSourcesPathBase = '../../../../../savina-lf-extra-variants'
numIterationsDefault = 10
numIterationsAkka = 10
numThreads_lf1 = 1
numThreads_lf2 = 2
numThreads_lf4 = 4
numThreads_lf8 = 8
numThreads_lf16 = 16

experiments = {}


runPingPongCmd = f'bin/PingPongBenchmark --fast --numIterations {numIterationsDefault} '
runPingPongAkkaCmd = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -iter {numIterationsAkka} -n'
experiments['PingPong'] = {
    'description': 'PingPong benchmark with a variable number of Pings, keep other parameters constant.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '1',
    'globalPlotXAxisLabel': 'PingPong (1 mil pings)',
    'plotTitle': 'PingPong benchmark',
    'plotXAxisLabel': 'Number of pings in millions',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '1': {
            'lf1': [ f'lfc {os.path.join(lfSourceFilePathBase, "pingpong/PingPongBenchmark.lf")}'.split(),
                   (runPingPongCmd + f'--threads {numThreads_lf1} --count 1000000').split() ],
            'lf2': [ f'lfc {os.path.join(lfSourceFilePathBase, "pingpong/PingPongBenchmark.lf")}'.split(),
                   (runPingPongCmd + f'--threads {numThreads_lf2} --count 1000000').split() ],
            'lf4': [ f'lfc {os.path.join(lfSourceFilePathBase, "pingpong/PingPongBenchmark.lf")}'.split(),
                   (runPingPongCmd + f'--threads {numThreads_lf4} --count 1000000').split() ],
            'lf8': [ f'lfc {os.path.join(lfSourceFilePathBase, "pingpong/PingPongBenchmark.lf")}'.split(),
                   (runPingPongCmd + f'--threads {numThreads_lf8} --count 1000000').split() ],
            'lf16': [ f'lfc {os.path.join(lfSourceFilePathBase, "pingpong/PingPongBenchmark.lf")}'.split(),
                   (runPingPongCmd + f'--threads {numThreads_lf16} --count 1000000').split() ],
            'savina-akka-default': [ f'{runPingPongAkkaCmd} 1000000'.split() ]
        },
        '2': {
            'lf1': [ (runPingPongCmd + f'--threads {numThreads_lf1} --count 2000000').split() ],
            'lf2': [ (runPingPongCmd + f'--threads {numThreads_lf2} --count 2000000').split() ],
            'lf4': [ (runPingPongCmd + f'--threads {numThreads_lf4} --count 2000000').split() ],
            'lf8': [ (runPingPongCmd + f'--threads {numThreads_lf8} --count 2000000').split() ],
            'lf16': [ (runPingPongCmd + f'--threads {numThreads_lf16} --count 2000000').split() ],
            'savina-akka-default': [ f'{runPingPongAkkaCmd} 2000000'.split() ]
        },
        '3': {
            'lf1': [ (runPingPongCmd + f'--threads {numThreads_lf1} --count 3000000').split() ],
            'lf2': [ (runPingPongCmd + f'--threads {numThreads_lf2} --count 3000000').split() ],
            'lf4': [ (runPingPongCmd + f'--threads {numThreads_lf4} --count 3000000').split() ],
            'lf8': [ (runPingPongCmd + f'--threads {numThreads_lf8} --count 3000000').split() ],
            'lf16': [ (runPingPongCmd + f'--threads {numThreads_lf16} --count 3000000').split() ],
            'savina-akka-default': [ f'{runPingPongAkkaCmd} 3000000'.split() ]
        },
        '4': {
            'lf1': [ (runPingPongCmd + f'--threads {numThreads_lf1} --count 4000000').split() ],
            'lf2': [ (runPingPongCmd + f'--threads {numThreads_lf2} --count 4000000').split() ],
            'lf4': [ (runPingPongCmd + f'--threads {numThreads_lf4} --count 4000000').split() ],
            'lf8': [ (runPingPongCmd + f'--threads {numThreads_lf8} --count 4000000').split() ],
            'lf16': [ (runPingPongCmd + f'--threads {numThreads_lf16} --count 4000000').split() ],
            'savina-akka-default': [ f'{runPingPongAkkaCmd} 4000000'.split() ]
        },
        '5': {
            'lf1': [ (runPingPongCmd + f'--threads {numThreads_lf1} --count 5000000').split() ],
            'lf2': [ (runPingPongCmd + f'--threads {numThreads_lf2} --count 5000000').split() ],
            'lf4': [ (runPingPongCmd + f'--threads {numThreads_lf4} --count 5000000').split() ],
            'lf8': [ (runPingPongCmd + f'--threads {numThreads_lf8} --count 5000000').split() ],
            'lf16': [ (runPingPongCmd + f'--threads {numThreads_lf16} --count 5000000').split() ],
            'savina-akka-default': [ f'{runPingPongAkkaCmd} 5000000'.split() ]
        },
        '6': {
            'lf1': [ (runPingPongCmd + f'--threads {numThreads_lf1} --count 6000000').split() ],
            'lf2': [ (runPingPongCmd + f'--threads {numThreads_lf2} --count 6000000').split() ],
            'lf4': [ (runPingPongCmd + f'--threads {numThreads_lf4} --count 6000000').split() ],
            'lf8': [ (runPingPongCmd + f'--threads {numThreads_lf8} --count 6000000').split() ],
            'lf16': [ (runPingPongCmd + f'--threads {numThreads_lf16} --count 6000000').split() ],
            'savina-akka-default': [ f'{runPingPongAkkaCmd} 6000000'.split() ]
        },
        '7': {
            'lf1': [ (runPingPongCmd + f'--threads {numThreads_lf1} --count 7000000').split() ],
            'lf2': [ (runPingPongCmd + f'--threads {numThreads_lf2} --count 7000000').split() ],
            'lf4': [ (runPingPongCmd + f'--threads {numThreads_lf4} --count 7000000').split() ],
            'lf8': [ (runPingPongCmd + f'--threads {numThreads_lf8} --count 7000000').split() ],
            'lf16': [ (runPingPongCmd + f'--threads {numThreads_lf16} --count 7000000').split() ],
            'savina-akka-default': [ f'{runPingPongAkkaCmd} 7000000'.split() ]
        },
        '8': {
            'lf1': [ (runPingPongCmd + f'--threads {numThreads_lf1} --count 8000000').split() ],
            'lf2': [ (runPingPongCmd + f'--threads {numThreads_lf2} --count 8000000').split() ],
            'lf4': [ (runPingPongCmd + f'--threads {numThreads_lf4} --count 8000000').split() ],
            'lf8': [ (runPingPongCmd + f'--threads {numThreads_lf8} --count 8000000').split() ],
            'lf16': [ (runPingPongCmd + f'--threads {numThreads_lf16} --count 8000000').split() ],
            'savina-akka-default': [ f'{runPingPongAkkaCmd} 8000000'.split() ]
        },
    }
}


runPingPongCmd = f'bin/PingPongBenchmark --fast --numIterations {numIterationsDefault} '
runPingPongAkkaCmd = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -iter {numIterationsAkka} -n'
experiments['PingPongWithC'] = {
    'description': 'PingPong benchmark with a variable number of Pings, keep other parameters constant.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '1',
    'globalPlotXAxisLabel': 'PingPong (1 mil pings)',
    'plotTitle': 'PingPong benchmark',
    'plotXAxisLabel': 'Number of pings in millions',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '1': {
            'lf-c-1': [ f'python -m cogapp -r -D numMessages=1000000 {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'lfc {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'for (( i=0 ; i<{numIterationsDefault} ; i++ )) ; do {os.path.join(externalLfSourcesPathBase, "bin/PingPongCGenerator")} --fast true --threads 1 ; done' ],
            'lf1': [ f'lfc {os.path.join(lfSourceFilePathBase, "pingpong/PingPongBenchmark.lf")}'.split(),
                   (runPingPongCmd + f'--threads {numThreads_lf1} --count 1000000').split() ],
            'lf2': [ f'lfc {os.path.join(lfSourceFilePathBase, "pingpong/PingPongBenchmark.lf")}'.split(),
                   (runPingPongCmd + f'--threads {numThreads_lf2} --count 1000000').split() ],
            'lf4': [ f'lfc {os.path.join(lfSourceFilePathBase, "pingpong/PingPongBenchmark.lf")}'.split(),
                   (runPingPongCmd + f'--threads {numThreads_lf4} --count 1000000').split() ],
            'lf8': [ f'lfc {os.path.join(lfSourceFilePathBase, "pingpong/PingPongBenchmark.lf")}'.split(),
                   (runPingPongCmd + f'--threads {numThreads_lf8} --count 1000000').split() ],
            'lf16': [ f'lfc {os.path.join(lfSourceFilePathBase, "pingpong/PingPongBenchmark.lf")}'.split(),
                   (runPingPongCmd + f'--threads {numThreads_lf16} --count 1000000').split() ],
            'savina-akka-default': [ f'{runPingPongAkkaCmd} 1000000'.split() ]
        },
        '2': {
            'lf-c-1': [ f'python -m cogapp -r -D numMessages=2000000 {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'lfc {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'for (( i=0 ; i<{numIterationsDefault} ; i++ )) ; do {os.path.join(externalLfSourcesPathBase, "bin/PingPongCGenerator")} --fast true --threads 1 ; done' ],
            'lf1': [ (runPingPongCmd + f'--threads {numThreads_lf1} --count 2000000').split() ],
            'lf2': [ (runPingPongCmd + f'--threads {numThreads_lf2} --count 2000000').split() ],
            'lf4': [ (runPingPongCmd + f'--threads {numThreads_lf4} --count 2000000').split() ],
            'lf8': [ (runPingPongCmd + f'--threads {numThreads_lf8} --count 2000000').split() ],
            'lf16': [ (runPingPongCmd + f'--threads {numThreads_lf16} --count 2000000').split() ],
            'savina-akka-default': [ f'{runPingPongAkkaCmd} 2000000'.split() ]
        },
        '3': {
            'lf-c-1': [ f'python -m cogapp -r -D numMessages=3000000 {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'lfc {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'for (( i=0 ; i<{numIterationsDefault} ; i++ )) ; do {os.path.join(externalLfSourcesPathBase, "bin/PingPongCGenerator")} --fast true --threads 1 ; done' ],
            'lf1': [ (runPingPongCmd + f'--threads {numThreads_lf1} --count 3000000').split() ],
            'lf2': [ (runPingPongCmd + f'--threads {numThreads_lf2} --count 3000000').split() ],
            'lf4': [ (runPingPongCmd + f'--threads {numThreads_lf4} --count 3000000').split() ],
            'lf8': [ (runPingPongCmd + f'--threads {numThreads_lf8} --count 3000000').split() ],
            'lf16': [ (runPingPongCmd + f'--threads {numThreads_lf16} --count 3000000').split() ],
            'savina-akka-default': [ f'{runPingPongAkkaCmd} 3000000'.split() ]
        },
        '4': {
            'lf-c-1': [ f'python -m cogapp -r -D numMessages=4000000 {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'lfc {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'for (( i=0 ; i<{numIterationsDefault} ; i++ )) ; do {os.path.join(externalLfSourcesPathBase, "bin/PingPongCGenerator")} --fast true --threads 1 ; done' ],
            'lf1': [ (runPingPongCmd + f'--threads {numThreads_lf1} --count 4000000').split() ],
            'lf2': [ (runPingPongCmd + f'--threads {numThreads_lf2} --count 4000000').split() ],
            'lf4': [ (runPingPongCmd + f'--threads {numThreads_lf4} --count 4000000').split() ],
            'lf8': [ (runPingPongCmd + f'--threads {numThreads_lf8} --count 4000000').split() ],
            'lf16': [ (runPingPongCmd + f'--threads {numThreads_lf16} --count 4000000').split() ],
            'savina-akka-default': [ f'{runPingPongAkkaCmd} 4000000'.split() ]
        },
        '5': {
            'lf-c-1': [ f'python -m cogapp -r -D numMessages=5000000 {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'lfc {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'for (( i=0 ; i<{numIterationsDefault} ; i++ )) ; do {os.path.join(externalLfSourcesPathBase, "bin/PingPongCGenerator")} --fast true --threads 1 ; done' ],
            'lf1': [ (runPingPongCmd + f'--threads {numThreads_lf1} --count 5000000').split() ],
            'lf2': [ (runPingPongCmd + f'--threads {numThreads_lf2} --count 5000000').split() ],
            'lf4': [ (runPingPongCmd + f'--threads {numThreads_lf4} --count 5000000').split() ],
            'lf8': [ (runPingPongCmd + f'--threads {numThreads_lf8} --count 5000000').split() ],
            'lf16': [ (runPingPongCmd + f'--threads {numThreads_lf16} --count 5000000').split() ],
            'savina-akka-default': [ f'{runPingPongAkkaCmd} 5000000'.split() ]
        },
        '6': {
            'lf-c-1': [ f'python -m cogapp -r -D numMessages=6000000 {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'lfc {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'for (( i=0 ; i<{numIterationsDefault} ; i++ )) ; do {os.path.join(externalLfSourcesPathBase, "bin/PingPongCGenerator")} --fast true --threads 1 ; done' ],
            'lf1': [ (runPingPongCmd + f'--threads {numThreads_lf1} --count 6000000').split() ],
            'lf2': [ (runPingPongCmd + f'--threads {numThreads_lf2} --count 6000000').split() ],
            'lf4': [ (runPingPongCmd + f'--threads {numThreads_lf4} --count 6000000').split() ],
            'lf8': [ (runPingPongCmd + f'--threads {numThreads_lf8} --count 6000000').split() ],
            'lf16': [ (runPingPongCmd + f'--threads {numThreads_lf16} --count 6000000').split() ],
            'savina-akka-default': [ f'{runPingPongAkkaCmd} 6000000'.split() ]
        },
        '7': {
            'lf-c-1': [ f'python -m cogapp -r -D numMessages=7000000 {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'lfc {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'for (( i=0 ; i<{numIterationsDefault} ; i++ )) ; do {os.path.join(externalLfSourcesPathBase, "bin/PingPongCGenerator")} --fast true --threads 1 ; done' ],
            'lf1': [ (runPingPongCmd + f'--threads {numThreads_lf1} --count 7000000').split() ],
            'lf2': [ (runPingPongCmd + f'--threads {numThreads_lf2} --count 7000000').split() ],
            'lf4': [ (runPingPongCmd + f'--threads {numThreads_lf4} --count 7000000').split() ],
            'lf8': [ (runPingPongCmd + f'--threads {numThreads_lf8} --count 7000000').split() ],
            'lf16': [ (runPingPongCmd + f'--threads {numThreads_lf16} --count 7000000').split() ],
            'savina-akka-default': [ f'{runPingPongAkkaCmd} 7000000'.split() ]
        },
        '8': {
            'lf-c-1': [ f'python -m cogapp -r -D numMessages=8000000 {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'lfc {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'for (( i=0 ; i<{numIterationsDefault} ; i++ )) ; do {os.path.join(externalLfSourcesPathBase, "bin/PingPongCGenerator")} --fast true --threads 1 ; done' ],
            'lf1': [ (runPingPongCmd + f'--threads {numThreads_lf1} --count 8000000').split() ],
            'lf2': [ (runPingPongCmd + f'--threads {numThreads_lf2} --count 8000000').split() ],
            'lf4': [ (runPingPongCmd + f'--threads {numThreads_lf4} --count 8000000').split() ],
            'lf8': [ (runPingPongCmd + f'--threads {numThreads_lf8} --count 8000000').split() ],
            'lf16': [ (runPingPongCmd + f'--threads {numThreads_lf16} --count 8000000').split() ],
            'savina-akka-default': [ f'{runPingPongAkkaCmd} 8000000'.split() ]
        },
    }
}


runThreadRingCmdLf1 = f'bin/ThreadRingBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} --numPings'
runThreadRingCmdLf2 = f'bin/ThreadRingBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} --numPings'
runThreadRingCmdLf4 = f'bin/ThreadRingBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} --numPings'
runThreadRingCmdLf8 = f'bin/ThreadRingBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} --numPings'
runThreadRingCmdLf16 = f'bin/ThreadRingBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} --numPings'
runThreadRingCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.threadring.ThreadRingAkkaActorBenchmark -iter {numIterationsAkka} -n 100 -r'
experiments['ThreadRingVarNumPings'] = {
    'description': 'Thread Ring from the Savina suite with 100 actors/reactors and different number of pings.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '1',
    'globalPlotXAxisLabel': 'ThreadRing (100 actors)',
    'plotTitle': 'ThreadRing benchmark (100 actors)',
    'plotXAxisLabel': 'Number of pings in millions',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '1': {
            'lf1': [ f'python -m cogapp -r -D numReactors=100 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf1} 1000000'.split() ],
            'lf2': [ f'python -m cogapp -r -D numReactors=100 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf2} 1000000'.split() ],
            'lf4': [ f'python -m cogapp -r -D numReactors=100 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf4} 1000000'.split() ],
            'lf8': [ f'python -m cogapp -r -D numReactors=100 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf8} 1000000'.split() ],
            'lf16': [ f'python -m cogapp -r -D numReactors=100 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf16} 1000000'.split() ],
            'savina-akka-default': [ f'{runThreadRingCmdAkka} 1000000'.split() ]
        },
        '2': {
            'lf1': [ f'{runThreadRingCmdLf1} 2000000'.split() ],
            'lf2': [ f'{runThreadRingCmdLf2} 2000000'.split() ],
            'lf4': [ f'{runThreadRingCmdLf4} 2000000'.split() ],
            'lf8': [ f'{runThreadRingCmdLf8} 2000000'.split() ],
            'lf16': [ f'{runThreadRingCmdLf16} 2000000'.split() ],
            'savina-akka-default': [ f'{runThreadRingCmdAkka} 2000000'.split() ]
        },
        '3': {
            'lf1': [ f'{runThreadRingCmdLf1} 3000000'.split() ],
            'lf2': [ f'{runThreadRingCmdLf2} 3000000'.split() ],
            'lf4': [ f'{runThreadRingCmdLf4} 3000000'.split() ],
            'lf8': [ f'{runThreadRingCmdLf8} 3000000'.split() ],
            'lf16': [ f'{runThreadRingCmdLf16} 3000000'.split() ],
            'savina-akka-default': [ f'{runThreadRingCmdAkka} 3000000'.split() ]
        },
        '4': {
            'lf1': [ f'{runThreadRingCmdLf1} 4000000'.split() ],
            'lf2': [ f'{runThreadRingCmdLf2} 4000000'.split() ],
            'lf4': [ f'{runThreadRingCmdLf4} 4000000'.split() ],
            'lf8': [ f'{runThreadRingCmdLf8} 4000000'.split() ],
            'lf16': [ f'{runThreadRingCmdLf16} 4000000'.split() ],
            'savina-akka-default': [ f'{runThreadRingCmdAkka} 4000000'.split() ]
        },
        '5': {
            'lf1': [ f'{runThreadRingCmdLf1} 5000000'.split() ],
            'lf2': [ f'{runThreadRingCmdLf2} 5000000'.split() ],
            'lf4': [ f'{runThreadRingCmdLf4} 5000000'.split() ],
            'lf8': [ f'{runThreadRingCmdLf8} 5000000'.split() ],
            'lf16': [ f'{runThreadRingCmdLf16} 5000000'.split() ],
            'savina-akka-default': [ f'{runThreadRingCmdAkka} 5000000'.split() ]
        },
        '6': {
            'lf1': [ f'{runThreadRingCmdLf1} 6000000'.split() ],
            'lf2': [ f'{runThreadRingCmdLf2} 6000000'.split() ],
            'lf4': [ f'{runThreadRingCmdLf4} 6000000'.split() ],
            'lf8': [ f'{runThreadRingCmdLf8} 6000000'.split() ],
            'lf16': [ f'{runThreadRingCmdLf16} 6000000'.split() ],
            'savina-akka-default': [ f'{runThreadRingCmdAkka} 6000000'.split() ]
        },
        '7': {
            'lf1': [ f'{runThreadRingCmdLf1} 7000000'.split() ],
            'lf2': [ f'{runThreadRingCmdLf2} 7000000'.split() ],
            'lf4': [ f'{runThreadRingCmdLf4} 7000000'.split() ],
            'lf8': [ f'{runThreadRingCmdLf8} 7000000'.split() ],
            'lf16': [ f'{runThreadRingCmdLf16} 7000000'.split() ],
            'savina-akka-default': [ f'{runThreadRingCmdAkka} 7000000'.split() ]
        },
        '8': {
            'lf1': [ f'{runThreadRingCmdLf1} 8000000'.split() ],
            'lf2': [ f'{runThreadRingCmdLf2} 8000000'.split() ],
            'lf4': [ f'{runThreadRingCmdLf4} 8000000'.split() ],
            'lf8': [ f'{runThreadRingCmdLf8} 8000000'.split() ],
            'lf16': [ f'{runThreadRingCmdLf16} 8000000'.split() ],
            'savina-akka-default': [ f'{runThreadRingCmdAkka} 8000000'.split() ]
        }
    }
}


numPings = 1000000
runThreadRingCmdLf1 = f'bin/ThreadRingBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} --numPings {numPings}'
runThreadRingCmdLf2 = f'bin/ThreadRingBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} --numPings {numPings}'
runThreadRingCmdLf4 = f'bin/ThreadRingBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} --numPings {numPings}'
runThreadRingCmdLf8 = f'bin/ThreadRingBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} --numPings {numPings}'
runThreadRingCmdLf16 = f'bin/ThreadRingBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} --numPings {numPings}'
runThreadRingCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.threadring.ThreadRingAkkaActorBenchmark -iter {numIterationsAkka} -r {numPings} -n'
experiments['ThreadRingVarActors'] = {
    'description': 'Thread Ring from the Savina suite with 1 million pings and a varying number of actors/reactors.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '100',
    'globalPlotXAxisLabel': 'Thread Ring (1 million pings)',
    'plotTitle': 'Thread Ring (1 million pings)',
    'plotXAxisLabel': 'Number of actors/reactors',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '100': {
            'lf1': [ f'python -m cogapp -r -D numReactors=100 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D numReactors=100 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D numReactors=100 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D numReactors=100 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D numReactors=100 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runThreadRingCmdAkka} 100'.split() ]
        },
        '200': {
            'lf1': [ f'python -m cogapp -r -D numReactors=200 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D numReactors=200 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D numReactors=200 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D numReactors=200 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D numReactors=200 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runThreadRingCmdAkka} 200'.split() ]
        },
        '300': {
            'lf1': [ f'python -m cogapp -r -D numReactors=300 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D numReactors=300 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D numReactors=300 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D numReactors=300 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D numReactors=300 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runThreadRingCmdAkka} 300'.split() ]
        },
        '400': {
            'lf1': [ f'python -m cogapp -r -D numReactors=400 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D numReactors=400 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D numReactors=400 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D numReactors=400 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D numReactors=400 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runThreadRingCmdAkka} 400'.split() ]
        },
        '500': {
            'lf1': [ f'python -m cogapp -r -D numReactors=500 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D numReactors=500 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D numReactors=500 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D numReactors=500 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D numReactors=500 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runThreadRingCmdAkka} 500'.split() ]
        },
        '600': {
            'lf1': [ f'python -m cogapp -r -D numReactors=600 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D numReactors=600 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D numReactors=600 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D numReactors=600 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D numReactors=600 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
                   f'{runThreadRingCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runThreadRingCmdAkka} 600'.split() ]
        }
    }
}


runCountingCmdLf1 = f'bin/CountingBenchmark --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} --countTo'
runCountingCmdLf2 = f'bin/CountingBenchmark --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} --countTo'
runCountingCmdLf4 = f'bin/CountingBenchmark --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} --countTo'
runCountingCmdLf8 = f'bin/CountingBenchmark --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} --countTo'
runCountingCmdLf16 = f'bin/CountingBenchmark --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} --countTo'
runCoutingCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.count.CountingAkkaActorBenchmark -iter {numIterationsAkka} -n'
experiments['CountingActor'] = {
    'description': 'Counting benchmark from the Savina suite with a variable count',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '1',
    'globalPlotXAxisLabel': 'ThreadRing',
    'plotTitle': 'Counting benchmark',
    'plotXAxisLabel': 'Counter in millions',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '1': {
            'lf1': [ f'lfc {os.path.join(lfSourceFilePathBase, "count/CountingBenchmark.lf")}'.split(),
                   f'{runCountingCmdLf1} 1000000'.split() ],
            'lf2': [ f'lfc {os.path.join(lfSourceFilePathBase, "count/CountingBenchmark.lf")}'.split(),
                   f'{runCountingCmdLf2} 1000000'.split() ],
            'lf4': [ f'lfc {os.path.join(lfSourceFilePathBase, "count/CountingBenchmark.lf")}'.split(),
                   f'{runCountingCmdLf4} 1000000'.split() ],
            'lf8': [ f'lfc {os.path.join(lfSourceFilePathBase, "count/CountingBenchmark.lf")}'.split(),
                   f'{runCountingCmdLf8} 1000000'.split() ],
            'lf16': [ f'lfc {os.path.join(lfSourceFilePathBase, "count/CountingBenchmark.lf")}'.split(),
                   f'{runCountingCmdLf16} 1000000'.split() ],
            'savina-akka-default': [ f'{runCoutingCmdAkka} 1000000'.split() ]
        },
        '2': {
            'lf1': [ f'{runCountingCmdLf1} 2000000'.split() ],
            'lf2': [ f'{runCountingCmdLf2} 2000000'.split() ],
            'lf4': [ f'{runCountingCmdLf4} 2000000'.split() ],
            'lf8': [ f'{runCountingCmdLf8} 2000000'.split() ],
            'lf16': [ f'{runCountingCmdLf16} 2000000'.split() ],
            'savina-akka-default': [ f'{runCoutingCmdAkka} 2000000'.split() ]
        },
        '3': {
            'lf1': [ f'{runCountingCmdLf1} 3000000'.split() ],
            'lf2': [ f'{runCountingCmdLf2} 3000000'.split() ],
            'lf4': [ f'{runCountingCmdLf4} 3000000'.split() ],
            'lf8': [ f'{runCountingCmdLf8} 3000000'.split() ],
            'lf16': [ f'{runCountingCmdLf16} 3000000'.split() ],
            'savina-akka-default': [ f'{runCoutingCmdAkka} 3000000'.split() ]
        },
        '4': {
            'lf1': [ f'{runCountingCmdLf1} 4000000'.split() ],
            'lf2': [ f'{runCountingCmdLf2} 4000000'.split() ],
            'lf4': [ f'{runCountingCmdLf4} 4000000'.split() ],
            'lf8': [ f'{runCountingCmdLf8} 4000000'.split() ],
            'lf16': [ f'{runCountingCmdLf16} 4000000'.split() ],
            'savina-akka-default': [ f'{runCoutingCmdAkka} 4000000'.split() ]
        },
        '5': {
            'lf1': [ f'{runCountingCmdLf1} 5000000'.split() ],
            'lf2': [ f'{runCountingCmdLf2} 5000000'.split() ],
            'lf4': [ f'{runCountingCmdLf4} 5000000'.split() ],
            'lf8': [ f'{runCountingCmdLf8} 5000000'.split() ],
            'lf16': [ f'{runCountingCmdLf16} 5000000'.split() ],
            'savina-akka-default': [ f'{runCoutingCmdAkka} 5000000'.split() ]
        },
        '6': {
            'lf1': [ f'{runCountingCmdLf1} 6000000'.split() ],
            'lf2': [ f'{runCountingCmdLf2} 6000000'.split() ],
            'lf4': [ f'{runCountingCmdLf4} 6000000'.split() ],
            'lf8': [ f'{runCountingCmdLf8} 6000000'.split() ],
            'lf16': [ f'{runCountingCmdLf16} 6000000'.split() ],
            'savina-akka-default': [ f'{runCoutingCmdAkka} 6000000'.split() ]
        },
        '7': {
            'lf1': [ f'{runCountingCmdLf1} 7000000'.split() ],
            'lf2': [ f'{runCountingCmdLf2} 7000000'.split() ],
            'lf4': [ f'{runCountingCmdLf4} 7000000'.split() ],
            'lf8': [ f'{runCountingCmdLf8} 7000000'.split() ],
            'lf16': [ f'{runCountingCmdLf16} 7000000'.split() ],
            'savina-akka-default': [ f'{runCoutingCmdAkka} 7000000'.split() ]
        },
        '8': {
            'lf1': [ f'{runCountingCmdLf1} 8000000'.split() ],
            'lf2': [ f'{runCountingCmdLf2} 8000000'.split() ],
            'lf4': [ f'{runCountingCmdLf4} 8000000'.split() ],
            'lf8': [ f'{runCountingCmdLf8} 8000000'.split() ],
            'lf16': [ f'{runCountingCmdLf16} 8000000'.split() ],
            'savina-akka-default': [ f'{runCoutingCmdAkka} 8000000'.split() ]
        }
    }
}


runCmdLf1 = f'bin/ThroughputBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} --numMessagesPerReactor'
runCmdLf2 = f'bin/ThroughputBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} --numMessagesPerReactor'
runCmdLf4 = f'bin/ThroughputBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} --numMessagesPerReactor'
runCmdLf8 = f'bin/ThroughputBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} --numMessagesPerReactor'
runCmdLf16 = f'bin/ThroughputBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} --numMessagesPerReactor'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.fjthrput.ThroughputAkkaActorBenchmark -iter {numIterationsAkka} -a 60 -n'
experiments['ThroughputVarNumMessages'] = {
    'description': 'Fork join (throughput) benchmark from the Savina suite with a variable number of messages',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '100',
    'globalPlotXAxisLabel': 'Throughput (60 workers)',
    'plotTitle': 'Fork join (throughput)',
    'plotXAxisLabel': 'Number of messages in thousands',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '100': {
            'lf1': [ f'python -m cogapp -r -D numWorkers=60 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf1} 100000'.split() ],
            'lf2': [ f'python -m cogapp -r -D numWorkers=60 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf2} 100000'.split() ],
            'lf4': [ f'python -m cogapp -r -D numWorkers=60 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf4} 100000'.split() ],
            'lf8': [ f'python -m cogapp -r -D numWorkers=60 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf8} 100000'.split() ],
            'lf16': [ f'python -m cogapp -r -D numWorkers=60 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                    f'{runCmdLf16} 100000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 100000'.split() ]
        },
        '200': {
            'lf1': [ f'{runCmdLf1} 200000'.split() ],
            'lf2': [ f'{runCmdLf2} 200000'.split() ],
            'lf4': [ f'{runCmdLf4} 200000'.split() ],
            'lf8': [ f'{runCmdLf8} 200000'.split() ],
            'lf16': [ f'{runCmdLf16} 200000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 200000'.split() ]
        },
        '300': {
            'lf1': [ f'{runCmdLf1} 300000'.split() ],
            'lf2': [ f'{runCmdLf2} 300000'.split() ],
            'lf4': [ f'{runCmdLf4} 300000'.split() ],
            'lf8': [ f'{runCmdLf8} 300000'.split() ],
            'lf16': [ f'{runCmdLf16} 300000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 300000'.split() ]
        }
    }
}


runCmdLf1 = f'bin/ThroughputBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} --numMessagesPerReactor 100000'
runCmdLf2 = f'bin/ThroughputBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} --numMessagesPerReactor 100000'
runCmdLf4 = f'bin/ThroughputBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} --numMessagesPerReactor 100000'
runCmdLf8 = f'bin/ThroughputBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} --numMessagesPerReactor 100000'
runCmdLf16 = f'bin/ThroughputBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} --numMessagesPerReactor 100000'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.fjthrput.ThroughputAkkaActorBenchmark -iter {numIterationsAkka} -n 100000 -a'
experiments['ThroughputVarNumWorkers'] = {
    'description': 'Fork join (throughput) benchmark from the Savina suite with a variable number workers',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '60',
    'globalPlotXAxisLabel': 'Throughput (100k messages)',
    'plotTitle': 'Fork join (throughput)',
    'plotXAxisLabel': 'Number of workers',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '60': {
            'lf1': [ f'python -m cogapp -r -D numWorkers=60 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D numWorkers=60 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D numWorkers=60 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D numWorkers=60 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D numWorkers=60 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                    f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 60'.split() ]
        },
        '120': {
            'lf1': [ f'python -m cogapp -r -D numWorkers=120 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D numWorkers=120 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D numWorkers=120 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D numWorkers=120 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D numWorkers=120 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                    f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 120'.split() ]
        },
        '180': {
            'lf1': [ f'python -m cogapp -r -D numWorkers=180 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D numWorkers=180 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D numWorkers=180 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D numWorkers=180 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D numWorkers=180 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                    f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 180'.split() ]
        },
        '240': {
            'lf1': [ f'python -m cogapp -r -D numWorkers=240 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D numWorkers=240 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D numWorkers=240 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D numWorkers=240 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                   f'{runCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D numWorkers=240 {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, "fjthrput/ThroughputBenchmarkGenerator.lf")}'.split(),
                    f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 240'.split() ]
        }
    }
}

binName = 'ChameneosBenchmarkGenerator'
lfSrcPath = f'chameneos/{binName}.lf'
constParam1 = 60
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} --numMeetings'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} --numMeetings'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} --numMeetings'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} --numMeetings'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} --numMeetings'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.fjthrput.ThroughputAkkaActorBenchmark -iter {numIterationsAkka} -numChameneos {constParam1} -numMeetings'
experiments['ChameneosVarNumMessages'] = {
    'description': f'Chameneos benchmark from the Savina suite with {constParam1} chameneos and a variable number of meetings.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '200',
    'globalPlotXAxisLabel': 'Chameneos (60 Chameneos)',
    'plotTitle': 'Chameneos',
    'plotXAxisLabel': 'Number of meetings in thousands',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '100': {
            'lf1': [ f'python -m cogapp -r -D numChameneos={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1} 100000'.split() ],
            'lf2': [ f'python -m cogapp -r -D numWorkers={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2} 100000'.split() ],
            'lf4': [ f'python -m cogapp -r -D numWorkers={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4} 100000'.split() ],
            'lf8': [ f'python -m cogapp -r -D numWorkers={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8} 100000'.split() ],
            'lf16': [ f'python -m cogapp -r -D numWorkers={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16} 100000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 100000'.split() ]
        },
        '200': {
            'lf1': [ f'{runCmdLf1} 200000'.split() ],
            'lf2': [ f'{runCmdLf2} 200000'.split() ],
            'lf4': [ f'{runCmdLf4} 200000'.split() ],
            'lf8': [ f'{runCmdLf8} 200000'.split() ],
            'lf16': [ f'{runCmdLf16} 200000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 200000'.split() ]
        },
        '300': {
            'lf1': [ f'{runCmdLf1} 300000'.split() ],
            'lf2': [ f'{runCmdLf2} 300000'.split() ],
            'lf4': [ f'{runCmdLf4} 300000'.split() ],
            'lf8': [ f'{runCmdLf8} 300000'.split() ],
            'lf16': [ f'{runCmdLf16} 300000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 300000'.split() ]
        },
        '400': {
            'lf1': [ f'{runCmdLf1} 400000'.split() ],
            'lf2': [ f'{runCmdLf2} 400000'.split() ],
            'lf4': [ f'{runCmdLf4} 400000'.split() ],
            'lf8': [ f'{runCmdLf8} 400000'.split() ],
            'lf16': [ f'{runCmdLf16} 400000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 400000'.split() ]
        },
        '500': {
            'lf1': [ f'{runCmdLf1} 500000'.split() ],
            'lf2': [ f'{runCmdLf2} 500000'.split() ],
            'lf4': [ f'{runCmdLf4} 500000'.split() ],
            'lf8': [ f'{runCmdLf8} 500000'.split() ],
            'lf16': [ f'{runCmdLf16} 500000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 500000'.split() ]
        },
        '600': {
            'lf1': [ f'{runCmdLf1} 600000'.split() ],
            'lf2': [ f'{runCmdLf2} 600000'.split() ],
            'lf4': [ f'{runCmdLf4} 600000'.split() ],
            'lf8': [ f'{runCmdLf8} 600000'.split() ],
            'lf16': [ f'{runCmdLf16} 600000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 600000'.split() ]
        },
        '700': {
            'lf1': [ f'{runCmdLf1} 700000'.split() ],
            'lf2': [ f'{runCmdLf2} 700000'.split() ],
            'lf4': [ f'{runCmdLf4} 700000'.split() ],
            'lf8': [ f'{runCmdLf8} 700000'.split() ],
            'lf16': [ f'{runCmdLf16} 700000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 700000'.split() ]
        },
        '800': {
            'lf1': [ f'{runCmdLf1} 800000'.split() ],
            'lf2': [ f'{runCmdLf2} 800000'.split() ],
            'lf4': [ f'{runCmdLf4} 800000'.split() ],
            'lf8': [ f'{runCmdLf8} 800000'.split() ],
            'lf16': [ f'{runCmdLf16} 800000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 800000'.split() ]
        }
    }
}


binName = 'BigBenchmarkGenerator'
lfSrcPath = f'big/{binName}.lf'
paramName1 = f'--numPingsPerReactor'
constParam1 = 20
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} {paramName1}'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} {paramName1}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} {paramName1}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} {paramName1}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} {paramName1}'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.big.BigAkkaActorBenchmark -iter {numIterationsAkka} -w {constParam1} -n'
experiments['BigVarNumPings'] = {
    'description': f'Big benchmark from the Savina suite with {constParam1} actors and a variable number of pings.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '20',
    'globalPlotXAxisLabel': f'Big ({constParam1} actors)',
    'plotTitle': f'Big ({constParam1} actors)',
    'plotXAxisLabel': 'Number of meetings in thousands',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '20': {
            'lf1': [ f'python -m cogapp -r -D numReactors={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1} 20000'.split() ],
            'lf2': [ f'python -m cogapp -r -D numReactors={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2} 20000'.split() ],
            'lf4': [ f'python -m cogapp -r -D numReactors={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4} 20000'.split() ],
            'lf8': [ f'python -m cogapp -r -D numReactors={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8} 20000'.split() ],
            'lf16': [ f'python -m cogapp -r -D numReactors={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16} 20000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 20000'.split() ]
        },
        '40': {
            'lf1': [ f'{runCmdLf1} 40000'.split() ],
            'lf2': [ f'{runCmdLf2} 40000'.split() ],
            'lf4': [ f'{runCmdLf4} 40000'.split() ],
            'lf8': [ f'{runCmdLf8} 40000'.split() ],
            'lf16': [ f'{runCmdLf16} 40000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 40000'.split() ]
        },
        '60': {
            'lf1': [ f'{runCmdLf1} 60000'.split() ],
            'lf2': [ f'{runCmdLf2} 60000'.split() ],
            'lf4': [ f'{runCmdLf4} 60000'.split() ],
            'lf8': [ f'{runCmdLf8} 60000'.split() ],
            'lf16': [ f'{runCmdLf16} 60000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 60000'.split() ]
        },
        '80': {
            'lf1': [ f'{runCmdLf1} 80000'.split() ],
            'lf2': [ f'{runCmdLf2} 80000'.split() ],
            'lf4': [ f'{runCmdLf4} 80000'.split() ],
            'lf8': [ f'{runCmdLf8} 80000'.split() ],
            'lf16': [ f'{runCmdLf16} 80000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 80000'.split() ]
        },
        '100': {
            'lf1': [ f'{runCmdLf1} 100000'.split() ],
            'lf2': [ f'{runCmdLf2} 100000'.split() ],
            'lf4': [ f'{runCmdLf4} 100000'.split() ],
            'lf8': [ f'{runCmdLf8} 100000'.split() ],
            'lf16': [ f'{runCmdLf16} 100000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 100000'.split() ]
        },
        '120': {
            'lf1': [ f'{runCmdLf1} 120000'.split() ],
            'lf2': [ f'{runCmdLf2} 120000'.split() ],
            'lf4': [ f'{runCmdLf4} 120000'.split() ],
            'lf8': [ f'{runCmdLf8} 120000'.split() ],
            'lf16': [ f'{runCmdLf16} 120000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 120000'.split() ]
        }
    }
}


binName = 'DictionaryBenchmarkGenerator'
lfSrcPath = f'concdic/{binName}.lf'
paramName1 = f'--numMessagesPerWorker'
constParam1 = 20
constParam2 = 10
cogParamName1 = 'numWorkers'
paramName2 = f'--writePercentage {constParam2}'
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} {paramName2} {paramName1}'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} {paramName2} {paramName1}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} {paramName2} {paramName1}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} {paramName2} {paramName1}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} {paramName2} {paramName1}'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.concdict.DictionaryAkkaActorBenchmark -iter {numIterationsAkka} -w {constParam2} -e {constParam1} -m'
experiments['DictionaryVarNumMsgsPerWorker'] = {
    'description': f'Benchmark concurrent dictionary from the Savina suite with {constParam1} actors and a variable number of messages per worker.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '20',
    'globalPlotXAxisLabel': f'Concurrent dictionary ({constParam1} workers)',
    'plotTitle': f'Concurrent dictionary ({constParam1} workers)',
    'plotXAxisLabel': 'Number of messages per worker in thousands',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '20': {
            'lf1': [ f'python -m cogapp -r -D {cogParamName1}={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1} 20000'.split() ],
            'lf2': [ f'python -m cogapp -r -D {cogParamName1}={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2} 20000'.split() ],
            'lf4': [ f'python -m cogapp -r -D {cogParamName1}={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4} 20000'.split() ],
            'lf8': [ f'python -m cogapp -r -D {cogParamName1}={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8} 20000'.split() ],
            'lf16': [ f'python -m cogapp -r -D {cogParamName1}={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16} 20000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 20000'.split() ]
        },
        '40': {
            'lf1': [ f'{runCmdLf1} 40000'.split() ],
            'lf2': [ f'{runCmdLf2} 40000'.split() ],
            'lf4': [ f'{runCmdLf4} 40000'.split() ],
            'lf8': [ f'{runCmdLf8} 40000'.split() ],
            'lf16': [ f'{runCmdLf16} 40000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 40000'.split() ]
        },
        '60': {
            'lf1': [ f'{runCmdLf1} 60000'.split() ],
            'lf2': [ f'{runCmdLf2} 60000'.split() ],
            'lf4': [ f'{runCmdLf4} 60000'.split() ],
            'lf8': [ f'{runCmdLf8} 60000'.split() ],
            'lf16': [ f'{runCmdLf16} 60000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 60000'.split() ]
        },
        '80': {
            'lf1': [ f'{runCmdLf1} 80000'.split() ],
            'lf2': [ f'{runCmdLf2} 80000'.split() ],
            'lf4': [ f'{runCmdLf4} 80000'.split() ],
            'lf8': [ f'{runCmdLf8} 80000'.split() ],
            'lf16': [ f'{runCmdLf16} 80000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 80000'.split() ]
        },
        '100': {
            'lf1': [ f'{runCmdLf1} 100000'.split() ],
            'lf2': [ f'{runCmdLf2} 100000'.split() ],
            'lf4': [ f'{runCmdLf4} 100000'.split() ],
            'lf8': [ f'{runCmdLf8} 100000'.split() ],
            'lf16': [ f'{runCmdLf16} 100000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 100000'.split() ]
        },
        '120': {
            'lf1': [ f'{runCmdLf1} 120000'.split() ],
            'lf2': [ f'{runCmdLf2} 120000'.split() ],
            'lf4': [ f'{runCmdLf4} 120000'.split() ],
            'lf8': [ f'{runCmdLf8} 120000'.split() ],
            'lf16': [ f'{runCmdLf16} 120000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 120000'.split() ]
        }
    }
}


binName = 'SortedListBenchmarkGenerator'
lfSrcPath = f'concsll/{binName}.lf'
paramName1 = f'--numMessagesPerWorker'
constParam1 = 20
constParam2 = 10
constParam3 = 1
cogParamName1 = 'numWorkers'
paramName2 = f'--writePercentage {constParam2}'
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} {paramName2} --sizePercentage {constParam3} {paramName1}'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} {paramName2} --sizePercentage {constParam3} {paramName1}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} {paramName2} --sizePercentage {constParam3} {paramName1}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} {paramName2} --sizePercentage {constParam3} {paramName1}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} {paramName2} --sizePercentage {constParam3} {paramName1}'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.concsll.SortedListAkkaActorBenchmark -iter {numIterationsAkka} -s {constParam3} -w {constParam2} -e {constParam1} -m'
experiments['SortedListVarNumMsgsPerWorker'] = {
    'description': f'Benchmark sorted linked list from the Savina suite with {constParam1} actors and a variable number of messages per worker.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '8',
    'globalPlotXAxisLabel': f'Sorted linked list ({constParam1} workers)',
    'plotTitle': f'Sorted linked list ({constParam1} workers)',
    'plotXAxisLabel': 'Number of messages per worker in thousands',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '8': {
            'lf1': [ f'python -m cogapp -r -D {cogParamName1}={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1} 8000'.split() ],
            'lf2': [ f'python -m cogapp -r -D {cogParamName1}={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2} 8000'.split() ],
            'lf4': [ f'python -m cogapp -r -D {cogParamName1}={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4} 8000'.split() ],
            'lf8': [ f'python -m cogapp -r -D {cogParamName1}={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8} 8000'.split() ],
            'lf16': [ f'python -m cogapp -r -D {cogParamName1}={constParam1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16} 8000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 8000'.split() ]
        },
        '16': {
            'lf1': [ f'{runCmdLf1} 16000'.split() ],
            'lf2': [ f'{runCmdLf2} 16000'.split() ],
            'lf4': [ f'{runCmdLf4} 16000'.split() ],
            'lf8': [ f'{runCmdLf8} 16000'.split() ],
            'lf16': [ f'{runCmdLf16} 16000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 16000'.split() ]
        },
        '24': {
            'lf1': [ f'{runCmdLf1} 24000'.split() ],
            'lf2': [ f'{runCmdLf2} 24000'.split() ],
            'lf4': [ f'{runCmdLf4} 24000'.split() ],
            'lf8': [ f'{runCmdLf8} 24000'.split() ],
            'lf16': [ f'{runCmdLf16} 24000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 24000'.split() ]
        },
        '32': {
            'lf1': [ f'{runCmdLf1} 32000'.split() ],
            'lf2': [ f'{runCmdLf2} 32000'.split() ],
            'lf4': [ f'{runCmdLf4} 32000'.split() ],
            'lf8': [ f'{runCmdLf8} 32000'.split() ],
            'lf16': [ f'{runCmdLf16} 32000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 32000'.split() ]
        },
        '40': {
            'lf1': [ f'{runCmdLf1} 40000'.split() ],
            'lf2': [ f'{runCmdLf2} 40000'.split() ],
            'lf4': [ f'{runCmdLf4} 40000'.split() ],
            'lf8': [ f'{runCmdLf8} 40000'.split() ],
            'lf16': [ f'{runCmdLf16} 40000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 40000'.split() ]
        },
        '48': {
            'lf1': [ f'{runCmdLf1} 48000'.split() ],
            'lf2': [ f'{runCmdLf2} 48000'.split() ],
            'lf4': [ f'{runCmdLf4} 48000'.split() ],
            'lf8': [ f'{runCmdLf8} 48000'.split() ],
            'lf16': [ f'{runCmdLf16} 48000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 48000'.split() ]
        }
    }
}



binName = 'ProdConsBenchmarkGenerator'
lfSrcPath = f'bndbuffer/{binName}.lf'
paramNameVar = f'--numItemsPerProducer'
paramNameLf1 = f'--bufferSize'
paramValue1 = 50
paramNameLf2 = f'--prodCost'
paramValue2 = 25
paramNameLf2 = f'--consCost'
paramValue2 = 25
paramNameLf3 = f'numProducers'
paramValue3 = 40
paramNameLf4 = f'numConsumers'
paramValue4 = 40
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameVar}'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameVar}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameVar}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameVar}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameVar}'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.bndbuffer.ProdConsAkkaActorBenchmark -iter {numIterationsAkka} -bb {paramValue1} -np {paramValue3} -nc {paramValue3} -pc {paramValue2} -cc {paramValue2} -ipp'
experiments['ProdConsVarNumItemsPerProducer'] = {
    'description': f'Benchmark producer-consumer from the Savina suite with {paramValue3} producers and {paramValue3} consumers and a variable number of items per producer.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '1',
    'globalPlotXAxisLabel': f'Producer-consumer ({paramValue3} producers, {paramValue4} consumers)',
    'plotTitle': f'Producer-consumer ({paramValue3} producers, {paramValue4} consumers)',
    'plotXAxisLabel': 'Number of items produced per producer in thousands',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '1': {
            'lf1': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} -D {paramNameLf4}={paramValue4} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1} 1000'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} -D {paramNameLf4}={paramValue4} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2} 1000'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} -D {paramNameLf4}={paramValue4} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4} 1000'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} -D {paramNameLf4}={paramValue4} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8} 1000'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} -D {paramNameLf4}={paramValue4} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16} 1000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 1000'.split() ]
        },
        '2': {
            'lf1': [ f'{runCmdLf1} 2000'.split() ],
            'lf2': [ f'{runCmdLf2} 2000'.split() ],
            'lf4': [ f'{runCmdLf4} 2000'.split() ],
            'lf8': [ f'{runCmdLf8} 2000'.split() ],
            'lf16': [ f'{runCmdLf16} 2000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 2000'.split() ]
        },
        '3': {
            'lf1': [ f'{runCmdLf1} 3000'.split() ],
            'lf2': [ f'{runCmdLf2} 3000'.split() ],
            'lf4': [ f'{runCmdLf4} 3000'.split() ],
            'lf8': [ f'{runCmdLf8} 3000'.split() ],
            'lf16': [ f'{runCmdLf16} 3000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 3000'.split() ]
        },
        '4': {
            'lf1': [ f'{runCmdLf1} 4000'.split() ],
            'lf2': [ f'{runCmdLf2} 4000'.split() ],
            'lf4': [ f'{runCmdLf4} 4000'.split() ],
            'lf8': [ f'{runCmdLf8} 4000'.split() ],
            'lf16': [ f'{runCmdLf16} 4000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 4000'.split() ]
        },
        '5': {
            'lf1': [ f'{runCmdLf1} 5000'.split() ],
            'lf2': [ f'{runCmdLf2} 5000'.split() ],
            'lf4': [ f'{runCmdLf4} 5000'.split() ],
            'lf8': [ f'{runCmdLf8} 5000'.split() ],
            'lf16': [ f'{runCmdLf16} 5000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 5000'.split() ]
        },
        '6': {
            'lf1': [ f'{runCmdLf1} 6000'.split() ],
            'lf2': [ f'{runCmdLf2} 6000'.split() ],
            'lf4': [ f'{runCmdLf4} 6000'.split() ],
            'lf8': [ f'{runCmdLf8} 6000'.split() ],
            'lf16': [ f'{runCmdLf16} 6000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 6000'.split() ]
        }
    }
}


binName = 'PhilosopherBenchmarkGenerator'
lfSrcPath = f'philosopher/{binName}.lf'
paramNameVar = f'--numEatingRounds'
paramNameLf1 = f'numPhilosophers'
paramValue1 = 20
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} {paramNameVar}'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} {paramNameVar}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} {paramNameVar}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} {paramNameVar}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} {paramNameVar}'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.philosopher.PhilosopherAkkaActorBenchmark -iter {numIterationsAkka} -n {paramValue1} -c 1 -m'
experiments['DiningPhilosophersVarNumEatingRounds'] = {
    'description': f'Benchmark dining philosophers from the Savina suite with {paramValue1} philosophers and a variable number of eating rounds.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '1',
    'globalPlotXAxisLabel': f'Dining Philosohpers ({paramValue1} philosophers)',
    'plotTitle': f'Dining Philosohpers ({paramValue1} philosophers)',
    'plotXAxisLabel': 'Number of eating rounds in thousands',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '1': {
            'lf1': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1} 1000'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2} 1000'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4} 1000'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8} 1000'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16} 1000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 1000'.split() ]
        },
        '2': {
            'lf1': [ f'{runCmdLf1} 2000'.split() ],
            'lf2': [ f'{runCmdLf2} 2000'.split() ],
            'lf4': [ f'{runCmdLf4} 2000'.split() ],
            'lf8': [ f'{runCmdLf8} 2000'.split() ],
            'lf16': [ f'{runCmdLf16} 2000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 2000'.split() ]
        },
        '3': {
            'lf1': [ f'{runCmdLf1} 3000'.split() ],
            'lf2': [ f'{runCmdLf2} 3000'.split() ],
            'lf4': [ f'{runCmdLf4} 3000'.split() ],
            'lf8': [ f'{runCmdLf8} 3000'.split() ],
            'lf16': [ f'{runCmdLf16} 3000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 3000'.split() ]
        },
        '4': {
            'lf1': [ f'{runCmdLf1} 4000'.split() ],
            'lf2': [ f'{runCmdLf2} 4000'.split() ],
            'lf4': [ f'{runCmdLf4} 4000'.split() ],
            'lf8': [ f'{runCmdLf8} 4000'.split() ],
            'lf16': [ f'{runCmdLf16} 4000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 4000'.split() ]
        },
        '5': {
            'lf1': [ f'{runCmdLf1} 5000'.split() ],
            'lf2': [ f'{runCmdLf2} 5000'.split() ],
            'lf4': [ f'{runCmdLf4} 5000'.split() ],
            'lf8': [ f'{runCmdLf8} 5000'.split() ],
            'lf16': [ f'{runCmdLf16} 5000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 5000'.split() ]
        },
        '6': {
            'lf1': [ f'{runCmdLf1} 6000'.split() ],
            'lf2': [ f'{runCmdLf2} 6000'.split() ],
            'lf4': [ f'{runCmdLf4} 6000'.split() ],
            'lf8': [ f'{runCmdLf8} 6000'.split() ],
            'lf16': [ f'{runCmdLf16} 6000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 6000'.split() ]
        }
    }
}


binName = 'SleepingBarberBenchmarkGenerator'
lfSrcPath = f'barber/{binName}.lf'
paramNameVar = f'numHaircuts'
paramNameLf1 = f'--waitingRoomSize'
paramValue1 = 1000
paramNameLf2 = f'--averageProductionRate'
paramValue2 = 1000
paramNameLf3 = f'--averageHaircutRate'
paramValue3 = 1000
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameLf3} {paramValue3}'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameLf3} {paramValue3}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameLf3} {paramValue3}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameLf3} {paramValue3}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameLf3} {paramValue3}'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.barber.SleepingBarberAkkaActorBenchmark -iter {numIterationsAkka} -w {paramValue1} -pr {paramValue2} -hr {paramValue3} -n'
experiments['SleepingBarberVarNumHaircuts'] = {
    'description': f'Benchmark sleeping barber from the Savina suite with a variable number of haircuts.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '1',
    'globalPlotXAxisLabel': f'Sleeping barber',
    'plotTitle': f'Sleeping barber',
    'plotXAxisLabel': 'Number of haircuts in thousands',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '1': {
            'lf1': [ f'python -m cogapp -r -D {paramNameVar}=1000 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameVar}=1000 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameVar}=1000 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameVar}=1000 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameVar}=1000 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 1000'.split() ]
        },
        '2': {
            'lf1': [ f'python -m cogapp -r -D {paramNameVar}=2000 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameVar}=2000 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameVar}=2000 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameVar}=2000 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameVar}=2000 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 2000'.split() ]
        },
        '3': {
            'lf1': [ f'python -m cogapp -r -D {paramNameVar}=3000 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameVar}=3000 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameVar}=3000 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameVar}=3000 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameVar}=3000 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 3000'.split() ]
        }
    }
}


binName = 'CigaretteSmokerBenchmarkGenerator'
lfSrcPath = f'cigsmok/{binName}.lf'
paramNameVar = f'--numRounds'
paramNameLf1 = f'numSmokers'
paramValue1 = 200
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} {paramNameVar}'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} {paramNameVar}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} {paramNameVar}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} {paramNameVar}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} {paramNameVar}'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.cigsmok.CigaretteSmokerAkkaActorBenchmark -iter {numIterationsAkka} -s {paramValue1} -r'
experiments['CigaretteSmokersVarNumRounds'] = {
    'description': f'Benchmark cigarette smokers from the Savina suite with {paramValue1} smokers and a variable number of rounds.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '1',
    'globalPlotXAxisLabel': f'Cigarette smokers ({paramValue1} smokers)',
    'plotTitle': f'Cigarette smokers ({paramValue1} smokers)',
    'plotXAxisLabel': 'Number of rounds in thousands',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '1': {
            'lf1': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1} 1000'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2} 1000'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4} 1000'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8} 1000'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16} 1000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 1000'.split() ]
        },
        '2': {
            'lf1': [ f'{runCmdLf1} 2000'.split() ],
            'lf2': [ f'{runCmdLf2} 2000'.split() ],
            'lf4': [ f'{runCmdLf4} 2000'.split() ],
            'lf8': [ f'{runCmdLf8} 2000'.split() ],
            'lf16': [ f'{runCmdLf16} 2000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 2000'.split() ]
        },
        '3': {
            'lf1': [ f'{runCmdLf1} 3000'.split() ],
            'lf2': [ f'{runCmdLf2} 3000'.split() ],
            'lf4': [ f'{runCmdLf4} 3000'.split() ],
            'lf8': [ f'{runCmdLf8} 3000'.split() ],
            'lf16': [ f'{runCmdLf16} 3000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 3000'.split() ]
        },
        '4': {
            'lf1': [ f'{runCmdLf1} 4000'.split() ],
            'lf2': [ f'{runCmdLf2} 4000'.split() ],
            'lf4': [ f'{runCmdLf4} 4000'.split() ],
            'lf8': [ f'{runCmdLf8} 4000'.split() ],
            'lf16': [ f'{runCmdLf16} 4000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 4000'.split() ]
        },
        '5': {
            'lf1': [ f'{runCmdLf1} 5000'.split() ],
            'lf2': [ f'{runCmdLf2} 5000'.split() ],
            'lf4': [ f'{runCmdLf4} 5000'.split() ],
            'lf8': [ f'{runCmdLf8} 5000'.split() ],
            'lf16': [ f'{runCmdLf16} 5000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 5000'.split() ]
        },
        '6': {
            'lf1': [ f'{runCmdLf1} 6000'.split() ],
            'lf2': [ f'{runCmdLf2} 6000'.split() ],
            'lf4': [ f'{runCmdLf4} 6000'.split() ],
            'lf8': [ f'{runCmdLf8} 6000'.split() ],
            'lf16': [ f'{runCmdLf16} 6000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 6000'.split() ]
        }
    }
}


binName = 'LogisticMapBenchmarkGenerator'
lfSrcPath = f'logmap/{binName}.lf'
paramNameVar = f'--numTerms'
paramNameLf1 = f'--startRate'
paramValue1 = '3.46'
paramNameLf2 = f'numSeries'
paramValue2 = 10
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} {paramNameLf1} {paramValue1} {paramNameVar}'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} {paramNameLf1} {paramValue1} {paramNameVar}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} {paramNameLf1} {paramValue1} {paramNameVar}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} {paramNameLf1} {paramValue1} {paramNameVar}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} {paramNameLf1} {paramValue1} {paramNameVar}'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.logmap.LogisticMapAkkaManualStashActorBenchmark -iter {numIterationsAkka} -s {paramValue2} -r {paramValue1} -t'
experiments['LogisticMapVarNumTerms'] = {
    'description': f'Benchmark logistic map from the Savina suite with {paramValue2} series and a variable number of terms.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '25',
    'globalPlotXAxisLabel': f'logistic map ({paramValue2} series)',
    'plotTitle': f'logistic map ({paramValue2} series)',
    'plotXAxisLabel': 'Number of terms in thousands',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '25': {
            'lf1': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1} 25000'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2} 25000'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4} 25000'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8} 25000'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16} 25000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 25000'.split() ]
        },
        '50': {
            'lf1': [ f'{runCmdLf1} 50000'.split() ],
            'lf2': [ f'{runCmdLf2} 50000'.split() ],
            'lf4': [ f'{runCmdLf4} 50000'.split() ],
            'lf8': [ f'{runCmdLf8} 50000'.split() ],
            'lf16': [ f'{runCmdLf16} 50000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 50000'.split() ]
        },
        '75': {
            'lf1': [ f'{runCmdLf1} 75000'.split() ],
            'lf2': [ f'{runCmdLf2} 75000'.split() ],
            'lf4': [ f'{runCmdLf4} 75000'.split() ],
            'lf8': [ f'{runCmdLf8} 75000'.split() ],
            'lf16': [ f'{runCmdLf16} 75000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 75000'.split() ]
        },
        '100': {
            'lf1': [ f'{runCmdLf1} 100000'.split() ],
            'lf2': [ f'{runCmdLf2} 100000'.split() ],
            'lf4': [ f'{runCmdLf4} 100000'.split() ],
            'lf8': [ f'{runCmdLf8} 100000'.split() ],
            'lf16': [ f'{runCmdLf16} 100000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 100000'.split() ]
        }
    }
}


binName = 'BankingBenchmarkGenerator'
lfSrcPath = f'banking/{binName}.lf'
paramNameVar = f'--numTransactions'
paramNameLf1 = f'numAccounts'
paramValue1 = 100
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} {paramNameVar}'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} {paramNameVar}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} {paramNameVar}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} {paramNameVar}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} {paramNameVar}'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.banking.BankingAkkaManualStashActorBenchmark -iter {numIterationsAkka} -a {paramValue1} -n'
experiments['BankingVarNumTransactions'] = {
    'description': f'Benchmark bank transactions from the Savina suite with {paramValue1} accounts and a variable number of transactions.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '10',
    'globalPlotXAxisLabel': f'Bank transaction ({paramValue1} accounts)',
    'plotTitle': f'Bank transaction ({paramValue1} accounts)',
    'plotXAxisLabel': 'Number of transactions in thousands',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '10': {
            'lf1': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1} 10000'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2} 10000'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4} 10000'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8} 10000'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16} 10000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 10000'.split() ]
        },
        '20': {
            'lf1': [ f'{runCmdLf1} 20000'.split() ],
            'lf2': [ f'{runCmdLf2} 20000'.split() ],
            'lf4': [ f'{runCmdLf4} 20000'.split() ],
            'lf8': [ f'{runCmdLf8} 20000'.split() ],
            'lf16': [ f'{runCmdLf16} 20000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 20000'.split() ]
        },
        '30': {
            'lf1': [ f'{runCmdLf1} 30000'.split() ],
            'lf2': [ f'{runCmdLf2} 30000'.split() ],
            'lf4': [ f'{runCmdLf4} 30000'.split() ],
            'lf8': [ f'{runCmdLf8} 30000'.split() ],
            'lf16': [ f'{runCmdLf16} 30000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 30000'.split() ]
        },
        '40': {
            'lf1': [ f'{runCmdLf1} 40000'.split() ],
            'lf2': [ f'{runCmdLf2} 40000'.split() ],
            'lf4': [ f'{runCmdLf4} 40000'.split() ],
            'lf8': [ f'{runCmdLf8} 40000'.split() ],
            'lf16': [ f'{runCmdLf16} 40000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 40000'.split() ]
        },
        '50': {
            'lf1': [ f'{runCmdLf1} 50000'.split() ],
            'lf2': [ f'{runCmdLf2} 50000'.split() ],
            'lf4': [ f'{runCmdLf4} 50000'.split() ],
            'lf8': [ f'{runCmdLf8} 50000'.split() ],
            'lf16': [ f'{runCmdLf16} 50000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 50000'.split() ]
        }
    }
}


binName = 'ApspBenchmarkGenerator'
lfSrcPath = f'apsp/{binName}.lf'
paramNameVar = f'numNodes'
paramNameLf1 = f'--maxEdgeWeight'
paramValue1 = 100
paramNameLf2 = f'blockSize'
paramValue2 = 50
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} {paramNameLf1} {paramValue1}'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} {paramNameLf1} {paramValue1}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} {paramNameLf1} {paramValue1}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} {paramNameLf1} {paramValue1}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} {paramNameLf1} {paramValue1}'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.apsp.ApspAkkaActorBenchmark -iter {numIterationsAkka} -b {paramValue2} -w {paramValue1} -n'
experiments['ApspVarNumNodes'] = {
    'description': f'Benchmark all pairs shortest paths from the Savina suite with block size {paramValue2} accounts and a variable number of nodes.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '3',
    'globalPlotXAxisLabel': f'Apsp (block size {paramValue2})',
    'plotTitle': f'Apsp (block size {paramValue2})',
    'plotXAxisLabel': 'Number of nodes in hundreds',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '3': {
            'lf1': [ f'python -m cogapp -r -D {paramNameVar}=300 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameVar}=300 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameVar}=300 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameVar}=300 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameVar}=300 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 300'.split() ]
        },
        '4': {
            'lf1': [ f'python -m cogapp -r -D {paramNameVar}=400 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameVar}=400 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameVar}=400 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameVar}=400 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameVar}=400 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 400'.split() ]
        },
        '5': {
            'lf1': [ f'python -m cogapp -r -D {paramNameVar}=500 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameVar}=500 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameVar}=500 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameVar}=500 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameVar}=500 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 500'.split() ]
        },
        '6': {
            'lf1': [ f'python -m cogapp -r -D {paramNameVar}=600 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameVar}=600 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameVar}=600 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameVar}=600 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameVar}=600 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 600'.split() ]
        },
        '7': {
            'lf1': [ f'python -m cogapp -r -D {paramNameVar}=700 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameVar}=700 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameVar}=700 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameVar}=700 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameVar}=700 -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 700'.split() ]
        }
    }
}


binName = 'GuidedSearchBenchmarkGenerator'
lfSrcPath = f'astar/{binName}.lf'
paramNameVar = f'--gridSize'
paramNameLf1 = f'--threshold'
paramValue1 = 1024
paramNameLf2 = f'--priorities'
paramValue2 = 30
paramNameLf3 = f'numWorkers'
paramValue3 = 20
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameVar}'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameVar}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameVar}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameVar}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameVar}'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.astar.GuidedSearchAkkaActorBenchmark -iter {numIterationsAkka} -w {paramValue3} -t {paramValue1} -p {paramValue2} -g'
experiments['AStarVarGridSize'] = {
    'description': f'Benchmark guided search from the Savina suite with {paramValue3} workers and a variable grid size.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '30',
    'globalPlotXAxisLabel': f'Guided search ({paramValue3} workers)',
    'plotTitle': f'Guided search ({paramValue3} workers)',
    'plotXAxisLabel': 'Grid size',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '30': {
            'lf1': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1} 30'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2} 30'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4} 30'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8} 30'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16} 30'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 30'.split() ]
        },
        '60': {
            'lf1': [ f'{runCmdLf1} 60'.split() ],
            'lf2': [ f'{runCmdLf2} 60'.split() ],
            'lf4': [ f'{runCmdLf4} 60'.split() ],
            'lf8': [ f'{runCmdLf8} 60'.split() ],
            'lf16': [ f'{runCmdLf16} 60'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 60'.split() ]
        },
        '90': {
            'lf1': [ f'{runCmdLf1} 90'.split() ],
            'lf2': [ f'{runCmdLf2} 90'.split() ],
            'lf4': [ f'{runCmdLf4} 90'.split() ],
            'lf8': [ f'{runCmdLf8} 90'.split() ],
            'lf16': [ f'{runCmdLf16} 90'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 90'.split() ]
        },
        '120': {
            'lf1': [ f'{runCmdLf1} 120'.split() ],
            'lf2': [ f'{runCmdLf2} 120'.split() ],
            'lf4': [ f'{runCmdLf4} 120'.split() ],
            'lf8': [ f'{runCmdLf8} 120'.split() ],
            'lf16': [ f'{runCmdLf16} 120'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 120'.split() ]
        },
        '150': {
            'lf1': [ f'{runCmdLf1} 150'.split() ],
            'lf2': [ f'{runCmdLf2} 150'.split() ],
            'lf4': [ f'{runCmdLf4} 150'.split() ],
            'lf8': [ f'{runCmdLf8} 150'.split() ],
            'lf16': [ f'{runCmdLf16} 150'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 150'.split() ]
        }
    }
}


binName = 'NQueensBenchmarkGenerator'
lfSrcPath = f'nqueenk/{binName}.lf'
paramNameVar = f'--size'
paramNameLf1 = f'--threshold'
paramValue1 = 4
paramNameLf2 = f'--solutionsLimit'
paramValue2 = 1500000
paramNameLf3 = f'numWorkers'
paramValue3 = 20
paramNameLf4 = f'--priorities'
paramValue4 = 10
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameLf4} {paramValue4} {paramNameVar}'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameLf4} {paramValue4} {paramNameVar}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameLf4} {paramValue4} {paramNameVar}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameLf4} {paramValue4} {paramNameVar}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameLf4} {paramValue4} {paramNameVar}'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.nqueenk.NQueensAkkaActorBenchmark -iter {numIterationsAkka} -t {paramValue1} -w {paramValue3} -s {paramValue2} -p {paramValue4} -n'
experiments['NQueensVarSize'] = {
    'description': f'Benchmark N queens first K solutions from the Savina suite with {paramValue3} workers and a variable size.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '12',
    'globalPlotXAxisLabel': f'N Queens ({paramValue3} workers)',
    'plotTitle': f'N Queens ({paramValue3} workers)',
    'plotXAxisLabel': 'Size',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '12': {
            'lf1': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1} 12'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2} 12'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4} 12'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8} 12'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16} 12'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 12'.split() ]
        },
        '13': {
            'lf1': [ f'{runCmdLf1} 13'.split() ],
            'lf2': [ f'{runCmdLf2} 13'.split() ],
            'lf4': [ f'{runCmdLf4} 13'.split() ],
            'lf8': [ f'{runCmdLf8} 13'.split() ],
            'lf16': [ f'{runCmdLf16} 13'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 13'.split() ]
        },
        '14': {
            'lf1': [ f'{runCmdLf1} 14'.split() ],
            'lf2': [ f'{runCmdLf2} 14'.split() ],
            'lf4': [ f'{runCmdLf4} 14'.split() ],
            'lf8': [ f'{runCmdLf8} 14'.split() ],
            'lf16': [ f'{runCmdLf16} 14'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 14'.split() ]
        },
        '15': {
            'lf1': [ f'{runCmdLf1} 15'.split() ],
            'lf2': [ f'{runCmdLf2} 15'.split() ],
            'lf4': [ f'{runCmdLf4} 15'.split() ],
            'lf8': [ f'{runCmdLf8} 15'.split() ],
            'lf16': [ f'{runCmdLf16} 15'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 15'.split() ]
        },
        '16': {
            'lf1': [ f'{runCmdLf1} 16'.split() ],
            'lf2': [ f'{runCmdLf2} 16'.split() ],
            'lf4': [ f'{runCmdLf4} 16'.split() ],
            'lf8': [ f'{runCmdLf8} 16'.split() ],
            'lf16': [ f'{runCmdLf16} 16'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 16'.split() ]
        }
    }
}


binName = 'MatMulBenchmarkGenerator'
lfSrcPath = f'recmatmul/{binName}.lf'
paramNameVar = f'--dataLength'
paramNameLf1 = f'--blockThreshold'
paramValue1 = 16384
paramNameLf2 = f'--priorities'
paramValue2 = 10
paramNameLf3 = f'numWorkers'
paramValue3 = 20
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameVar}'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameVar}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameVar}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameVar}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2} {paramNameVar}'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.recmatmul.MatMulAkkaActorBenchmark -iter {numIterationsAkka} -t {paramValue1} -w {paramValue3} -p {paramValue2} -n'
experiments['MatMulVarDataLength'] = {
    'description': f'Benchmark recursive matrix multiplication solutions from the Savina suite with {paramValue3} workers and a variable matrix size.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '1024',
    'globalPlotXAxisLabel': f'Matrix multiplication ({paramValue3} workers)',
    'plotTitle': f'Matrix multiplication ({paramValue3} workers)',
    'plotXAxisLabel': 'Number of rows',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '256': {
            'lf1': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1} 256'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2} 256'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4} 256'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8} 256'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameLf3}={paramValue3} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16} 256'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 256'.split() ]
        },
        '512': {
            'lf1': [ f'{runCmdLf1} 512'.split() ],
            'lf2': [ f'{runCmdLf2} 512'.split() ],
            'lf4': [ f'{runCmdLf4} 512'.split() ],
            'lf8': [ f'{runCmdLf8} 512'.split() ],
            'lf16': [ f'{runCmdLf16} 512'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 512'.split() ]
        },
        '1024': {
            'lf1': [ f'{runCmdLf1} 1024'.split() ],
            'lf2': [ f'{runCmdLf2} 1024'.split() ],
            'lf4': [ f'{runCmdLf4} 1024'.split() ],
            'lf8': [ f'{runCmdLf8} 1024'.split() ],
            'lf16': [ f'{runCmdLf16} 1024'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 1024'.split() ]
        },
        '2048': {
            'lf1': [ f'{runCmdLf1} 2048'.split() ],
            'lf2': [ f'{runCmdLf2} 2048'.split() ],
            'lf4': [ f'{runCmdLf4} 2048'.split() ],
            'lf8': [ f'{runCmdLf8} 2048'.split() ],
            'lf16': [ f'{runCmdLf16} 2048'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 2048'.split() ]
        },
        '4096': {
            'lf1': [ f'{runCmdLf1} 16'.split() ],
            'lf2': [ f'{runCmdLf2} 16'.split() ],
            'lf4': [ f'{runCmdLf4} 16'.split() ],
            'lf8': [ f'{runCmdLf8} 16'.split() ],
            'lf16': [ f'{runCmdLf16} 16'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 16'.split() ]
        }
    }
}


binName = 'RadixSortBenchmark'
lfSrcPath = f'radixsort/{binName}.lf'
paramNameVar = f'--dataSize'
paramNameLf1 = f'--seed'
paramValue1 = '2048'
paramNameLf2 = f'maxValue'
paramValue2 = '1152921504606846976' # 1 << 60
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} {paramNameLf1} {paramValue1} {paramNameVar}'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} {paramNameLf1} {paramValue1} {paramNameVar}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} {paramNameLf1} {paramValue1} {paramNameVar}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} {paramNameLf1} {paramValue1} {paramNameVar}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} {paramNameLf1} {paramValue1} {paramNameVar}'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.radixsort.RadixSortAkkaActorBenchmark -iter {numIterationsAkka} -s {paramValue1} -n'
experiments['RadixSortVarDataSize'] = {
    'description': f'Benchmark radix sort from the Savina suite with a variable data size.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '100',
    'globalPlotXAxisLabel': f'Radix sort',
    'plotTitle': f'Radix sort',
    'plotXAxisLabel': 'Number of data elements to sort in thousands',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '100': {
            'lf1': [ f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1} 100000'.split() ],
            'lf2': [ f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2} 100000'.split() ],
            'lf4': [ f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4} 100000'.split() ],
            'lf8': [ f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8} 100000'.split() ],
            'lf16': [ f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16} 100000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 100000'.split() ]
        },
        '200': {
            'lf1': [ f'{runCmdLf1} 200000'.split() ],
            'lf2': [ f'{runCmdLf2} 200000'.split() ],
            'lf4': [ f'{runCmdLf4} 200000'.split() ],
            'lf8': [ f'{runCmdLf8} 200000'.split() ],
            'lf16': [ f'{runCmdLf16} 200000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 200000'.split() ]
        },
        '300': {
            'lf1': [ f'{runCmdLf1} 300000'.split() ],
            'lf2': [ f'{runCmdLf2} 300000'.split() ],
            'lf4': [ f'{runCmdLf4} 300000'.split() ],
            'lf8': [ f'{runCmdLf8} 300000'.split() ],
            'lf16': [ f'{runCmdLf16} 300000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 300000'.split() ]
        },
        '400': {
            'lf1': [ f'{runCmdLf1} 400000'.split() ],
            'lf2': [ f'{runCmdLf2} 400000'.split() ],
            'lf4': [ f'{runCmdLf4} 400000'.split() ],
            'lf8': [ f'{runCmdLf8} 400000'.split() ],
            'lf16': [ f'{runCmdLf16} 400000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 400000'.split() ]
        },
        '500': {
            'lf1': [ f'{runCmdLf1} 500000'.split() ],
            'lf2': [ f'{runCmdLf2} 500000'.split() ],
            'lf4': [ f'{runCmdLf4} 500000'.split() ],
            'lf8': [ f'{runCmdLf8} 500000'.split() ],
            'lf16': [ f'{runCmdLf16} 500000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 500000'.split() ]
        }
    }
}


binName = 'FilterBankBenchmarkGenerator'
lfSrcPath = f'filterbank/{binName}.lf'
paramNameVar = f'numChannels'
paramNameLf1 = f'--numColumns'
paramValue1 = 16384
paramNameLf2 = f'--numSimulations'
paramValue2 = 34816
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2}'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} {paramNameLf1} {paramValue1} {paramNameLf2} {paramValue2}'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.filterbank.FilterBankAkkaActorBenchmark -iter {numIterationsAkka} -columns {paramValue1} -simulation {paramValue2}-channels'
experiments['FilterBankVarNumChannels'] = {
    'description': f'Benchmark filter bank from the Savina suite with a varying number of channels.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '8',
    'globalPlotXAxisLabel': f'Filter bank',
    'plotTitle': f'Filter bank',
    'plotXAxisLabel': 'Number of channels',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '8': {
            'lf1': [ f'python -m cogapp -r -D {paramNameVar}=8 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameVar}=8 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameVar}=8 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameVar}=8 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameVar}=8 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 8'.split() ]
        },
        '9': {
            'lf1': [ f'python -m cogapp -r -D {paramNameVar}=9 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameVar}=9 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameVar}=9 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameVar}=9 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameVar}=9 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 9'.split() ]
        },
        '10': {
            'lf1': [ f'python -m cogapp -r -D {paramNameVar}=10 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameVar}=10 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameVar}=10 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameVar}=10 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameVar}=10 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 10'.split() ]
        },
        '11': {
            'lf1': [ f'python -m cogapp -r -D {paramNameVar}=11 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameVar}=11 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameVar}=11 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameVar}=11 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameVar}=11 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 11'.split() ]
        },
        '12': {
            'lf1': [ f'python -m cogapp -r -D {paramNameVar}=12 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1}'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameVar}=12 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2}'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameVar}=12 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4}'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameVar}=12 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8}'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameVar}=12 {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 12'.split() ]
        }
    }
}


binName = 'TrapezoidalBenchmarkGenerator'
lfSrcPath = f'trapezoid/{binName}.lf'
paramNameVar = f'--numPieces'
paramNameLf1 = f'--leftEndPoint'
paramValue1 = '1.0'
paramNameLf2 = f'numWorkers'
paramValue2 = 100
paramNameLf3 = f'--rightEndPoint'
paramValue3 = '5.0'
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} {paramNameLf1} {paramValue1} {paramNameLf3} {paramValue3} {paramNameVar}'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} {paramNameLf1} {paramValue1} {paramNameLf3} {paramValue3} {paramNameVar}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} {paramNameLf1} {paramValue1} {paramNameLf3} {paramValue3} {paramNameVar}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} {paramNameLf1} {paramValue1} {paramNameLf3} {paramValue3} {paramNameVar}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} {paramNameLf1} {paramValue1} {paramNameLf3} {paramValue3} {paramNameVar}'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.trapezoid.TrapezoidalAkkaActorBenchmark -iter {numIterationsAkka} -w {paramValue2} -l {paramValue1} -r {paramValue3} -n'
experiments['TrapezoidalVarNumPieces'] = {
    'description': f'Benchmark trapezoidal approximation from the Savina suite with a variable number of pieces.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '10',
    'globalPlotXAxisLabel': f'Trapezoidal approximation ({paramValue2} workers)',
    'plotTitle': f'Trapezoidal approximation ({paramValue2} workers)',
    'plotXAxisLabel': 'Number of pieces in millions',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '10': {
            'lf1': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1} 10000000'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2} 10000000'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4} 10000000'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8} 10000000'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16} 10000000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 10000000'.split() ]
        },
        '20': {
            'lf1': [ f'{runCmdLf1} 20000000'.split() ],
            'lf2': [ f'{runCmdLf2} 20000000'.split() ],
            'lf4': [ f'{runCmdLf4} 20000000'.split() ],
            'lf8': [ f'{runCmdLf8} 20000000'.split() ],
            'lf16': [ f'{runCmdLf16} 20000000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 20000000'.split() ]
        },
        '30': {
            'lf1': [ f'{runCmdLf1} 30000000'.split() ],
            'lf2': [ f'{runCmdLf2} 30000000'.split() ],
            'lf4': [ f'{runCmdLf4} 30000000'.split() ],
            'lf8': [ f'{runCmdLf8} 30000000'.split() ],
            'lf16': [ f'{runCmdLf16} 30000000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 30000000'.split() ]
        },
        '40': {
            'lf1': [ f'{runCmdLf1} 40000000'.split() ],
            'lf2': [ f'{runCmdLf2} 40000000'.split() ],
            'lf4': [ f'{runCmdLf4} 40000000'.split() ],
            'lf8': [ f'{runCmdLf8} 40000000'.split() ],
            'lf16': [ f'{runCmdLf16} 40000000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 40000000'.split() ]
        },
        '50': {
            'lf1': [ f'{runCmdLf1} 50000000'.split() ],
            'lf2': [ f'{runCmdLf2} 50000000'.split() ],
            'lf4': [ f'{runCmdLf4} 50000000'.split() ],
            'lf8': [ f'{runCmdLf8} 50000000'.split() ],
            'lf16': [ f'{runCmdLf16} 50000000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 50000000'.split() ]
        }
    }
}


binName = 'PiPrecisionBenchmarkGenerator'
lfSrcPath = f'piprecision/{binName}.lf'
paramNameVar = f'--precision'
paramNameLf1 = f'numWorkers'
paramValue1 = 20
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} {paramNameVar}'
runCmdLf2 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} {paramNameVar}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} {paramNameVar}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} {paramNameVar}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} {paramNameVar}'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.piprecision.PiPrecisionAkkaActorBenchmark -iter {numIterationsAkka} -w {paramValue1} -p'
experiments['PiPrecisionVarPrecisionWithGmpLib'] = {
    'description': f'Benchmark pi precision from the Savina suite with a variable precision using the external library GMP.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '5',
    'globalPlotXAxisLabel': f'Pi Precision ({paramValue1} workers)',
    'plotTitle': f'Pi Precision ({paramValue1} workers)',
    'plotXAxisLabel': 'Precision in thousand digits',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'sequences': {
        '5': {
            'lf1': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf1} 5000'.split() ],
            'lf2': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf2} 5000'.split() ],
            'lf4': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf4} 5000'.split() ],
            'lf8': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                   f'{runCmdLf8} 5000'.split() ],
            'lf16': [ f'python -m cogapp -r -D {paramNameLf1}={paramValue1} {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'lfc {os.path.join(lfSourceFilePathBase, lfSrcPath)}'.split(),
                    f'{runCmdLf16} 5000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 5000'.split() ]
        },
        '7': {
            'lf1': [ f'{runCmdLf1} 7000'.split() ],
            'lf2': [ f'{runCmdLf2} 7000'.split() ],
            'lf4': [ f'{runCmdLf4} 7000'.split() ],
            'lf8': [ f'{runCmdLf8} 7000'.split() ],
            'lf16': [ f'{runCmdLf16} 7000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 7000'.split() ]
        },
        '9': {
            'lf1': [ f'{runCmdLf1} 9000'.split() ],
            'lf2': [ f'{runCmdLf2} 9000'.split() ],
            'lf4': [ f'{runCmdLf4} 9000'.split() ],
            'lf8': [ f'{runCmdLf8} 9000'.split() ],
            'lf16': [ f'{runCmdLf16} 9000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 9000'.split() ]
        },
        '11': {
            'lf1': [ f'{runCmdLf1} 11000'.split() ],
            'lf2': [ f'{runCmdLf2} 11000'.split() ],
            'lf4': [ f'{runCmdLf4} 11000'.split() ],
            'lf8': [ f'{runCmdLf8} 11000'.split() ],
            'lf16': [ f'{runCmdLf16} 11000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 11000'.split() ]
        },
        '13': {
            'lf1': [ f'{runCmdLf1} 13000'.split() ],
            'lf2': [ f'{runCmdLf2} 13000'.split() ],
            'lf4': [ f'{runCmdLf4} 13000'.split() ],
            'lf8': [ f'{runCmdLf8} 13000'.split() ],
            'lf16': [ f'{runCmdLf16} 13000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 13000'.split() ]
        }
    }
}








