import os.path

parsers = {
    'savina-akka': 'savina_parser',
    'lf': 'lf_parser',
    'lf-mt': 'lf_parser'
}

summarizers = {
    'savina-akka': 'default_summarizer',
    'lf': 'default_summarizer',
    'lf-mt': 'default_summarizer'
}

sequenceColors = {
    'savina-akka': 'violet',
    'lf': 'dark-turquoise',
    'lf-mt': 'skyblue'
}

# specs for global overview plot
globalPlot = {
    'plotTitle': 'Overview for all benchmarks',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'additionalGnuplotHeaderCommands': ''
}

# names for the sequences
sequenceNames = {
    'lf': 'Lingua Franca Cpp (1 thread)',
    'lf-mt': 'Lingua Franca Cpp (4 threads)',
    'savina-akka': 'Savina default config'
}

# helper and convenience variables
lfSourceFilePathBase = '..'
savinaJarPath = '../../../../../../savina/target/savina-0.0.1-SNAPSHOT-jar-with-dependencies.jar'
savinaPackagePathBase = 'edu.rice.habanero.benchmarks'
numIterationsDefault = 12
numThreadsSingle = 1
numThreadsMulti = 4

runPingPongCmd = f'bin/PingPongBenchmark --fast --numIterations {numIterationsDefault} '

experiments = {}

experiments['PingPong'] = {
    'description': 'Run the benchmark PingPong with a variable number of Pings, keep other parameters constant.',
    'additionalGnuplotHeaderCommands': '',
    'sequenceParameterForGlobalPlot': '1', # parameter value for the overview
    'globalPlotXAxisLabel': 'PingPong (1 mil pings)', # keep short
    'plotTitle': 'PingPong benchmark',
    'plotXAxisLabel': 'Number of pings in millions',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceTitles': { 
        'lf': 'Lingua Franca Cpp (1 thread)',
        'lf-mt': 'Lingua Franca Cpp (4 threads)',
        'savina-akka': 'Savina default config' },
    'sequences': {
        '1': {
            'lf': [ f'lfc {os.path.join(lfSourceFilePathBase, "pingpong/PingPongBenchmark.lf")}'.split(),
                   (runPingPongCmd + f'--threads {numThreadsSingle} --count 1000000').split() ],
            'lf-mt': [ f'lfc {os.path.join(lfSourceFilePathBase, "pingpong/PingPongBenchmark.lf")}'.split(),
                   (runPingPongCmd + f'--threads {numThreadsMulti} --count 1000000').split() ],
            'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -n 1000000'.split() ]
        },
        '2': {
            'lf': [ (runPingPongCmd + f'--threads {numThreadsSingle} --count 2000000').split() ],
            'lf-mt': [ (runPingPongCmd + f'--threads {numThreadsMulti} --count 2000000').split() ],
            'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -n 2000000'.split() ]
        },
        '3': {
            'lf': [ (runPingPongCmd + f'--threads {numThreadsSingle} --count 3000000').split() ],
            'lf-mt': [ (runPingPongCmd + f'--threads {numThreadsMulti} --count 3000000').split() ],
            'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -n 3000000'.split() ]
        },
        '4': {
            'lf': [ (runPingPongCmd + f'--threads {numThreadsSingle} --count 4000000').split() ],
            'lf-mt': [ (runPingPongCmd + f'--threads {numThreadsMulti} --count 4000000').split() ],
            'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -n 4000000'.split() ]
        },
        '5': {
            'lf': [ (runPingPongCmd + f'--threads {numThreadsSingle} --count 5000000').split() ],
            'lf-mt': [ (runPingPongCmd + f'--threads {numThreadsMulti} --count 5000000').split() ],
            'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -n 5000000'.split() ]
        },
        '6': {
            'lf': [ (runPingPongCmd + f'--threads {numThreadsSingle} --count 6000000').split() ],
            'lf-mt': [ (runPingPongCmd + f'--threads {numThreadsMulti} --count 6000000').split() ],
            'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -n 6000000'.split() ]
        },
        '7': {
            'lf': [ (runPingPongCmd + f'--threads {numThreadsSingle} --count 7000000').split() ],
            'lf-mt': [ (runPingPongCmd + f'--threads {numThreadsMulti} --count 7000000').split() ],
            'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -n 7000000'.split() ]
        },
        '8': {
            'lf': [ (runPingPongCmd + f'--threads {numThreadsSingle} --count 8000000').split() ],
            'lf-mt': [ (runPingPongCmd + f'--threads {numThreadsMulti} --count 8000000').split() ],
            'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark -n 8000000'.split() ]
        }
    }
}


#     'ThreadRingVarNumPings': {
#         'description': 'Thread Ring from the Savina suite.',
#         'additionalGnuplotHeaderCommands': '',
#         'sequenceParameterForGlobalPlot': '100', # which single point to use for the global plot
#         'globalPlotXAxisLabel': 'ThreadRing', # keep short
#         'plotTitle': 'ThreadRing benchmark',
#         'plotXAxisLabel': 'Number of pings in thousands',
#         'plotYAxisLabel': 'Execution time in ms (median)',
#         'plotSequenceTitles': { 
#             'lf': 'Lingua Franca Cpp (1 thread)',
#             'lf-mt': 'Lingua Franca Cpp (4 threads)',
#             'savina-akka': 'Savina default config' },
#         'sequences': {
#             '100': {
#                 'lf': [ f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmark.lf")}'.split(),
#                        (f'bin/ThreadRingBenchmark --fast --numIterations {numIterationsDefault} --threads {numThreadsSingle} --numPings 100000').split() ],
#                 'lf-mt': [ f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmark.lf")}'.split(),
#                        (f'bin/ThreadRingBenchmark --fast --numIterations {numIterationsDefault} --threads {numThreadsMulti} --numPings 100000').split() ],
#                 'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.threadring.ThreadRingAkkaActorBenchmark -n 10 -r 100000'.split() ]
#             }
#         }
#     },
#     'ThreadRingVarActors': {
#         'description': 'Thread Ring from the Savina suite.',
#         'additionalGnuplotHeaderCommands': '',
#         'sequenceParameterForGlobalPlot': '100', # which single point to use for the global plot
#         'globalPlotXAxisLabel': 'ThreadRing', # keep short
#         'plotTitle': 'ThreadRing benchmark',
#         'plotXAxisLabel': 'Number of actors',
#         'plotYAxisLabel': 'Execution time in ms (median)',
#         'plotSequenceTitles': { 
#             'lf': 'Lingua Franca Cpp (1 thread)',
#             'lf-mt': 'Lingua Franca Cpp (4 threads)',
#             'savina-akka': 'Savina default config' },
#         'sequences': {
#             '100': {
#                 'lf': [ f'python -m cogapp -r -D numReactors=100 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
#                        f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
#                        (f'bin/ThreadRingBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreadsSingle} --numPings 500000').split() ],
#                 'lf-mt': [ f'python -m cogapp -r -D numReactors=100 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
#                        f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
#                        (f'bin/ThreadRingBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreadsMulti} --numPings 500000').split() ],
#                 'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.threadring.ThreadRingAkkaActorBenchmark -n 100 -r 500000'.split() ]
#             },
#             '200': {
#                 'lf': [ f'python -m cogapp -r -D numReactors=200 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
#                        f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
#                        (f'bin/ThreadRingBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreadsSingle} --numPings 500000').split() ],
#                 'lf-mt': [ f'python -m cogapp -r -D numReactors=200 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
#                        f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
#                        (f'bin/ThreadRingBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreadsMulti} --numPings 500000').split() ],
#                 'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.threadring.ThreadRingAkkaActorBenchmark -n 200 -r 500000'.split() ]
#             },
#             '300': {
#                 'lf': [ f'python -m cogapp -r -D numReactors=300 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
#                        f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
#                        (f'bin/ThreadRingBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreadsSingle} --numPings 500000').split() ],
#                 'lf-mt': [ f'python -m cogapp -r -D numReactors=300 {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
#                        f'lfc {os.path.join(lfSourceFilePathBase, "threadring/ThreadRingBenchmarkGenerator.lf")}'.split(),
#                        (f'bin/ThreadRingBenchmarkGenerator --fast --numIterations {numIterationsDefault} --threads {numThreadsMulti} --numPings 500000').split() ],
#                 'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.threadring.ThreadRingAkkaActorBenchmark -n 300 -r 500000'.split() ]
#             }
#         }
#     },
#     'CountingActor': {
#         'description': 'Counting benchmark from the Savina suite.',
#         'additionalGnuplotHeaderCommands': '',
#         'sequenceParameterForGlobalPlot': '1', # which single point to use for the global plot
#         'globalPlotXAxisLabel': 'ThreadRing', # keep short
#         'plotTitle': 'Counting benchmark',
#         'plotXAxisLabel': 'Counter in millions',
#         'plotYAxisLabel': 'Execution time in ms (median)',
#         'plotSequenceTitles': { 
#             'lf': 'Lingua Franca Cpp (1 thread)',
#             'lf-mt': 'Lingua Franca Cpp (4 threads)',
#             'savina-akka': 'Savina default config' },
#         'sequences': {
#             '1': {
#                 'lf': [ f'lfc {os.path.join(lfSourceFilePathBase, "count/CountingBenchmark.lf")}'.split(),
#                        (f'bin/CountingBenchmark --fast --numIterations {numIterationsDefault} --threads {numThreadsSingle} --countTo 1000000').split() ],
#                 'lf-mt': [ f'lfc {os.path.join(lfSourceFilePathBase, "count/CountingBenchmark.lf")}'.split(),
#                        (f'bin/CountingBenchmark --fast --numIterations {numIterationsDefault} --threads {numThreadsMulti} --countTo 1000000').split() ],
#                 'savina-akka': [ f'java -classpath {savinaJarPath} {savinaPackagePathBase}.count.CountingAkkaActorBenchmark -n 1000000'.split() ]
#             }
#         }
#     }
# }

