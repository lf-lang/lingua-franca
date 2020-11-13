# This config is intended for a comparison between
# the Lingua Franca C++ implementation with different number of threads
# and the Savina Akka implementation.
#
# The experiment are not supposed to produce line diagrams but
# simple boxes.

import os

# variables that describe the environment and interface

# Where are the .lf files of the Savina Cpp implementation?
lfCppSourceFilePathBase = '../../benchmark/Cpp/Savina'

# Path to the jar file from the Savina benchmark suite:
savinaJarPath = './savina-0.0.1-SNAPSHOT-jar-with-dependencies.jar'

# The package path inside of Savina to the benchmarks:
savinaPackagePathBase = 'edu.rice.habanero.benchmarks'


# Options for the benchmarks:
numIterationsDefault = 4
numIterationsAkka = 4





# predefined global variables for convenience
parsers = {
    'savina-akka-default': 'savina_parser',
    'lf-cpp-1': 'lf_parser',
    'lf-cpp-2': 'lf_parser',
    'lf-cpp-4': 'lf_parser',
    'lf-cpp-8': 'lf_parser',
    'lf-cpp-16': 'lf_parser',
    'lf-c-1': 'lf_c_parser'
}
summarizers = {
    'savina-akka-default': 'summarizer_median_warmup',
    'lf-cpp-1': 'summarizer_median_warmup',
    'lf-cpp-2': 'summarizer_median_warmup',
    'lf-cpp-4': 'summarizer_median_warmup',
    'lf-cpp-8': 'summarizer_median_warmup',
    'lf-cpp-16': 'summarizer_median_warmup',
    'lf-c-1': 'summarizer_median_warmup'
}
plotter = 'plotterBox'
sequenceColors = {
    'savina-akka-default': '1',
    'lf-cpp-1': '2',
    'lf-cpp-2': '3',
    'lf-cpp-4': '4',
    'lf-cpp-8': '5',
    'lf-cpp-16': '6',
    'lf-c-1': '7'
}
sequenceNames = {
    'lf-cpp-1': 'LF C++ (1 thread)',
    'lf-cpp-2': 'LF C++ (2 threads)',
    'lf-cpp-4': 'LF C++ (4 threads)',
    'lf-cpp-8': 'LF C++ (8 threads)',
    'lf-cpp-16': 'LF C++ (16 threads)',
    'savina-akka-default': 'Akka (default config)',
    'lf-c-1': 'LF C (1 thread)'
}
arrangementSequences = [
    'lf-cpp-16',
    'lf-cpp-8',
    'lf-cpp-4',
    'lf-cpp-2',
    'lf-cpp-1',
    'lf-c-1',
    'savina-akka-default'
]

# parameters for the different benchmarks to run
numThreadsLfCpp1 = 1
numThreadsLfCpp2 = 2
numThreadsLfCpp4 = 4
numThreadsLfCpp8 = 8
numThreadsLfCpp16 = 16


# variables that are interpreted by the runner
globalPlot = {
    'plotTitle': 'Overview for all benchmarks',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotAdditionalGnuplotHeaderCommands': ''
}


experiments = {}



binName = 'PingPongBenchmark'
lfSrcPath = f'pingpong/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark'
paramNameLf1 = f'--count'
paramValue1 = 1000000
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {paramNameLf1} {paramValue1}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {paramNameLf1} {paramValue1}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {paramNameLf1} {paramValue1}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {paramNameLf1} {paramValue1}'
runCmdAkka = f'java -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -n {paramValue1}'
experiments['PingPong'] = {
    'description': f'Benchmark Ping Pong from the Savina suite with {paramValue1} pings.',
    'plotAdditionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '1',
    'globalPlotXAxisLabel': f'Ping Pong',
    'plotTitle': f'Ping Pong ({paramValue1} Pings)',
    'plotXAxisLabel': 'no value',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'plotter': plotter,
    'parsers': parsers,
    'summarizers': summarizers,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf4.split() ],
            'lf-cpp-8': [ f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf8.split() ],
            'lf-cpp-16': [ f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 300'.split() ]
        })
    ]
}


binName = 'ThreadRingBenchmarkGenerator'
lfSrcPath = f'threadring/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.threadring.ThreadRingAkkaActorBenchmark'
paramNameLf1 = f'--numPings'
paramValue1 = 1000000
paramNameLf2 = f'numReactors'
paramValue2 = 100
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {paramNameLf1} {paramValue1}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {paramNameLf1} {paramValue1}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {paramNameLf1} {paramValue1}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {paramNameLf1} {paramValue1}'
runCmdAkka = f'java -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -n {paramValue2} -r {paramValue1}'
experiments['ThreadRing'] = {
    'description': f'Thread Ring from the Savina suite with {paramValue2} actors/reactors.',
    'plotAdditionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '1',
    'globalPlotXAxisLabel': f'Ping Pong',
    'plotTitle': f'Thread Ring',
    'plotXAxisLabel': 'no value',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'plotter': plotter,
    'parsers': parsers,
    'summarizers': summarizers,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf4.split() ],
            'lf-cpp-8': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf8.split() ],
            'lf-cpp-16': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}


binName = 'CountingBenchmark'
lfSrcPath = f'count/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.count.CountingAkkaActorBenchmark'
paramNameLf1 = f'--countTo'
paramValue1 = 2000000
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {paramNameLf1} {paramValue1}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {paramNameLf1} {paramValue1}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {paramNameLf1} {paramValue1}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {paramNameLf1} {paramValue1}'
runCmdAkka = f'java -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -n {paramValue1}'
experiments['Counting'] = {
    'description': f'Counting benchmark from the Savina suite.',
    'plotAdditionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '1',
    'globalPlotXAxisLabel': f'Ping Pong',
    'plotTitle': f'Counting',
    'plotXAxisLabel': 'no value',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'plotter': plotter,
    'parsers': parsers,
    'summarizers': summarizers,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf4.split() ],
            'lf-cpp-8': [ f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf8.split() ],
            'lf-cpp-16': [ f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}


binName = 'ThroughputBenchmarkGenerator'
lfSrcPath = f'fjthrput/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.fjthrput.ThroughputAkkaActorBenchmark'
paramNameLf1 = f'--numMessagesPerReactor'
paramValue1 = 250000
paramNameLf2 = f'numWorkers'
paramValue2 = 60
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {paramNameLf1} {paramValue1}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {paramNameLf1} {paramValue1}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {paramNameLf1} {paramValue1}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {paramNameLf1} {paramValue1}'
runCmdAkka = f'java -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -a {paramValue2} -n {paramValue1}'
experiments['Throughput'] = {
    'description': f'Fork join (throughput) benchmark from the Savina suite.',
    'plotAdditionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '1',
    'globalPlotXAxisLabel': f'Ping Pong',
    'plotTitle': f'Fork Join (throughput)',
    'plotXAxisLabel': 'no value',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'plotter': plotter,
    'parsers': parsers,
    'summarizers': summarizers,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf4.split() ],
            'lf-cpp-8': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf8.split() ],
            'lf-cpp-16': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}


binName = 'ChameneosBenchmarkGenerator'
lfSrcPath = f'chameneos/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.chameneos.ChameneosAkkaActorBenchmark'
paramNameLf1 = f'--numMeetings'
paramValue1 = 500000
paramNameLf2 = f'numChameneos'
paramValue2 = 100
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {paramNameLf1} {paramValue1}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {paramNameLf1} {paramValue1}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {paramNameLf1} {paramValue1}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {paramNameLf1} {paramValue1}'
runCmdAkka = f'java -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -numChameneos {paramValue2} -numMeetings {paramValue1}'
experiments['Chameneos'] = {
    'description': f'Chameneos benchmark from the Savina suite.',
    'plotAdditionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '1',
    'globalPlotXAxisLabel': f'Ping Pong',
    'plotTitle': f'Chameneos',
    'plotXAxisLabel': 'no value',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'plotter': plotter,
    'parsers': parsers,
    'summarizers': summarizers,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf4.split() ],
            'lf-cpp-8': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf8.split() ],
            'lf-cpp-16': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}



binName = 'BigBenchmarkGenerator'
lfSrcPath = f'big/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.big.BigAkkaActorBenchmark'
paramNameLf1 = f'--numPingsPerReactor'
paramValue1 = 20000
paramNameLf2 = f'numReactors'
paramValue2 = 15
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {paramNameLf1} {paramValue1}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {paramNameLf1} {paramValue1}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {paramNameLf1} {paramValue1}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {paramNameLf1} {paramValue1}'
runCmdAkka = f'java -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -w {paramValue2} -n {paramValue1}'
experiments['Big'] = {
    'description': f'Big benchmark from the Savina suite.',
    'plotAdditionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '1',
    'globalPlotXAxisLabel': f'Ping Pong',
    'plotTitle': f'Big',
    'plotXAxisLabel': 'no value',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'plotter': plotter,
    'parsers': parsers,
    'summarizers': summarizers,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf4.split() ],
            'lf-cpp-8': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf8.split() ],
            'lf-cpp-16': [ f'python -m cogapp -r -D {paramNameLf2}={paramValue2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'lfc {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}




