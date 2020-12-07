# The config file specifies how to run benchmarks and plot the results.
# First, you need to provide parsers (for parsing the output of benchmarks)
# and summarizers (for summarizing the measurements into statistical values for plotting)
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
    'savina-akka-default': 'parserSavina',
    'lf1': 'parserLfCpp',
    'lf2': 'parserLfCpp',
    'lf4': 'parserLfCpp',
    'lf8': 'parserLfCpp',
    'lf16': 'parserLfCpp',
    'lf-c-1': 'parserLfC'
}

summarizers = {
    'savina-akka-default': 'summarizerMedianWarmup',
    'lf1': 'summarizerMedianWarmup',
    'lf2': 'summarizerMedianWarmup',
    'lf4': 'summarizerMedianWarmup',
    'lf8': 'summarizerMedianWarmup',
    'lf16': 'summarizerMedianWarmup',
    'lf-c-1': 'summarizerMedianWarmup'
}

plotter = 'plotterDefault'

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
    'plotAdditionalGnuplotHeaderCommands': ''
}

# names for the sequences
sequenceNames = {
    'lf1': 'LF1',
    'lf2': 'LF2',
    'lf4': 'LF4',
    'lf8': 'LF8',
    'lf16': 'LF 16',
    'savina-akka-default': 'AK',
    'lf-c-1': 'LF C (1 thread)'
}

# helper and convenience variables
lfSourceFilePathBase = '..'
savinaJarPath = '../../../../../../savina/target/savina-0.0.1-SNAPSHOT-jar-with-dependencies.jar'
savinaSrcPathBase = '../../../../../../savina'
savinaPackagePathBase = 'edu.rice.habanero.benchmarks'
externalLfSourcesPathBase = '../../../../../savina-lf-extra-variants'
numIterationsDefault = 4
numIterationsAkka = 4
numThreads_lf1 = 1
numThreads_lf2 = 2
numThreads_lf4 = 4
numThreads_lf8 = 8
numThreads_lf16 = 16


experiments = {}


runPingPongCmd = f'bin/PingPongBenchmark --fast --numIterations {numIterationsDefault} --count 1000000 --threads'
runPingPongAkkaCmd = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -iter {numIterationsAkka} -n 1000000'
experiments['PingPongVarNumThreads'] = {
    'description': 'PingPong benchmark with a variable number of threads, 1 million ping pongs.',
    'plotAdditionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '1',
    'globalPlotXAxisLabel': 'Ping Pong (1 mil ping pongs)',
    'plotTitle': 'Ping Pong benchmark',
    'plotXAxisLabel': 'Number of threads',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': { 'lf-cpp': 'violet', 'lf-c': 'dark-turquoise', 'savina-akka': 'skyblue' },
    'plotSequenceNames': { 'lf-cpp': 'LF C++', 'lf-c': 'LF C', 'savina-akka': 'Savina Akka' },
    'plotter': plotter,
    'parsers': { 'lf-cpp': 'parserLfCpp', 'lf-c': 'parserLfC', 'savina-akka': 'parserSavina' },
    'summarizers': { 'lf-cpp': 'summarizerMedianWarmup', 'lf-c': 'summarizerMedianWarmup', 'savina-akka': 'summarizerMedianWarmup' },
    'sequences': [
        ('1', {
            'lf-cpp': [ f'lfc {os.path.join(lfSourceFilePathBase, "pingpong/PingPongBenchmark.lf")}'.split(),
                       f'{runPingPongCmd} 1'.split() ],
            'lf-c': [ f'python -m cogapp -r -D numMessages=1000000 {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'lfc {os.path.join(externalLfSourcesPathBase, "PingPongCGenerator.lf")}'.split(),
                   f'for (( i=0 ; i<{numIterationsDefault} ; i++ )) ; do {os.path.join(externalLfSourcesPathBase, "bin/PingPongCGenerator")} --fast true --threads 1 ; done' ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=1 {os.path.join(savinaSrcPathBase, "src/main/scala/edu/rice/habanero/actors/AkkaActor.scala")}'.split(),
                            f'mvn package -f {savinaSrcPathBase}'.split(),
                            f'{runPingPongAkkaCmd}'.split() ]
        }),
        ('2', {
            'lf-cpp': [ f'{runPingPongCmd} 2'.split() ],
            'lf-c': [ f'for (( i=0 ; i<{numIterationsDefault} ; i++ )) ; do {os.path.join(externalLfSourcesPathBase, "bin/PingPongCGenerator")} --fast true --threads 2 ; done' ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=2 {os.path.join(savinaSrcPathBase, "src/main/scala/edu/rice/habanero/actors/AkkaActor.scala")}'.split(),
                            f'mvn package {savinaSrcPathBase}'.split(),
                            f'{runPingPongAkkaCmd}'.split() ]
        }),
        ('3', {
            'lf-cpp': [ f'{runPingPongCmd} 3'.split() ],
            'lf-c': [ f'for (( i=0 ; i<{numIterationsDefault} ; i++ )) ; do {os.path.join(externalLfSourcesPathBase, "bin/PingPongCGenerator")} --fast true --threads 3 ; done' ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=3 {os.path.join(savinaSrcPathBase, "src/main/scala/edu/rice/habanero/actors/AkkaActor.scala")}'.split(),
                            f'mvn package {savinaSrcPathBase}'.split(),
                            f'{runPingPongAkkaCmd}'.split() ]
        }),
        ('4', {
            'lf-cpp': [ f'{runPingPongCmd} 4'.split() ],
            'lf-c': [ f'for (( i=0 ; i<{numIterationsDefault} ; i++ )) ; do {os.path.join(externalLfSourcesPathBase, "bin/PingPongCGenerator")} --fast true --threads 4 ; done' ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=4 {os.path.join(savinaSrcPathBase, "src/main/scala/edu/rice/habanero/actors/AkkaActor.scala")}'.split(),
                            f'mvn package {savinaSrcPathBase}'.split(),
                            f'{runPingPongAkkaCmd}'.split() ]
        }),
        ('8', {
            'lf-cpp': [ f'{runPingPongCmd} 8'.split() ],
            'lf-c': [ f'for (( i=0 ; i<{numIterationsDefault} ; i++ )) ; do {os.path.join(externalLfSourcesPathBase, "bin/PingPongCGenerator")} --fast true --threads 8 ; done' ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=8 {os.path.join(savinaSrcPathBase, "src/main/scala/edu/rice/habanero/actors/AkkaActor.scala")}'.split(),
                            f'mvn package {savinaSrcPathBase}'.split(),
                            f'{runPingPongAkkaCmd}'.split() ]
        }),
        ('16', {
            'lf-cpp': [ f'{runPingPongCmd} 16'.split() ],
            'lf-c': [ f'for (( i=0 ; i<{numIterationsDefault} ; i++ )) ; do {os.path.join(externalLfSourcesPathBase, "bin/PingPongCGenerator")} --fast true --threads 16 ; done' ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=16 {os.path.join(savinaSrcPathBase, "src/main/scala/edu/rice/habanero/actors/AkkaActor.scala")}'.split(),
                            f'mvn package {savinaSrcPathBase}'.split(),
                            f'{runPingPongAkkaCmd}'.split() ]
        }),
        ('32', {
            'lf-cpp': [ f'{runPingPongCmd} 32'.split() ],
            'lf-c': [ f'for (( i=0 ; i<{numIterationsDefault} ; i++ )) ; do {os.path.join(externalLfSourcesPathBase, "bin/PingPongCGenerator")} --fast true --threads 32 ; done' ],
            'savina-akka': [ f'python -m cogapp -r -D numThreads=32 {os.path.join(savinaSrcPathBase, "src/main/scala/edu/rice/habanero/actors/AkkaActor.scala")}'.split(),
                            f'mvn package {savinaSrcPathBase}'.split(),
                            f'{runPingPongAkkaCmd}'.split() ]
        })
    ]
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
    'plotAdditionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '100',
    'globalPlotXAxisLabel': 'Thread Ring (1 million pings)',
    'plotTitle': 'Thread Ring (1 million pings)',
    'plotXAxisLabel': 'Number of actors/reactors',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotter': plotter,
    'parsers': parsers,
    'summarizers': summarizers,
    'sequences': [
        ('100', {
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
        }),
        ('200', {
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
        }),
        ('300', {
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
        }),
        ('400', {
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
        }),
        ('500', {
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
        }),
        ('600', {
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
        })
    ]
}



runCmdLf1 = f'bin/ThroughputBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf1} --numMessagesPerReactor 100000'
runCmdLf2 = f'bin/ThroughputBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf2} --numMessagesPerReactor 100000'
runCmdLf4 = f'bin/ThroughputBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf4} --numMessagesPerReactor 100000'
runCmdLf8 = f'bin/ThroughputBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf8} --numMessagesPerReactor 100000'
runCmdLf16 = f'bin/ThroughputBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreads_lf16} --numMessagesPerReactor 100000'
runCmdAkka = f'java -classpath {savinaJarPath} {savinaPackagePathBase}.fjthrput.ThroughputAkkaActorBenchmark -iter {numIterationsAkka} -n 100000 -a'
experiments['ThroughputVarNumWorkers'] = {
    'description': 'Fork join (throughput) benchmark from the Savina suite with a variable number workers',
    'plotAdditionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '60',
    'globalPlotXAxisLabel': 'Throughput (100k messages)',
    'plotTitle': 'Fork join (throughput)',
    'plotXAxisLabel': 'Number of workers',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotter': plotter,
    'parsers': parsers,
    'summarizers': summarizers,
    'sequences': [
        ('60', {
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
        }),
        ('120', {
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
        }),
        ('180', {
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
        }),
        ('240', {
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
        })
    ]
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
    'plotAdditionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '8',
    'globalPlotXAxisLabel': f'Filter bank',
    'plotTitle': f'Filter bank',
    'plotXAxisLabel': 'Number of channels',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotter': plotter,
    'parsers': parsers,
    'summarizers': summarizers,
    'sequences': [
        ('8', {
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
        }),
        ('9', {
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
        }),
        ('10', {
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
        }),
        ('11', {
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
        }),
        ('12', {
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
        })
    ]
}





