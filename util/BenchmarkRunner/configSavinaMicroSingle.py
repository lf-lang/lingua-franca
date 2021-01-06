# This config is runs the micro-benchmarks from Savina.
#
# Each experiment only has one sequence parameter, they are
# intended to produce box plots and not line diagrams.
#
# It runs the Akka implementation and LF C++ implementations.
#
# Recommended plotter module: plotterBoxerrorbar

import os


########### INTERFACE ########################################################
# Change parameters listet here according to your environment and likings.
# All paths are relative to the current working directory when running
# this script.
#
# Requirements:
# - Python module cog, install with 'pip3 install cogapp'
# - java version 1.8 for Savina, test with 'java -version'
# - Savina repository from https://github.com/shamsimam/savina
# - Executable lfc in PATH.

# Where are the .lf files of the Savina Cpp implementation?
lfCppSourceFilePathBase = '../../benchmark/Cpp/Savina'

# Path to the jar file from the Savina benchmark suite:
savinaJarPath = '../../../savina-original-var-threads/target/savina-0.0.1-SNAPSHOT-jar-with-dependencies.jar'

# The package path inside of Savina to the benchmarks:
savinaPackagePathBase = 'edu.rice.habanero.benchmarks'

# Executables to use
pythonExe = 'python3'
lfcExe = 'lfc'
javaExe = 'java'

# Options for the benchmarks:
numIterationsDefault = 12
numIterationsAkka = 12
########### INTERFACE ########################################################




# predefined global variables for convenience
parsers = {
    'savina-akka-default': 'parserSavina',
    'lf-cpp-1': 'parserLfCpp',
    'lf-cpp-2': 'parserLfCpp',
    'lf-cpp-4': 'parserLfCpp',
    'lf-cpp-8': 'parserLfCpp',
    'lf-cpp-16': 'parserLfCpp',
    'lf-c-1': 'parserLfC'
}
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
    'lf-cpp-1': 'LF1',
    'lf-cpp-2': 'LF2',
    'lf-cpp-4': 'LF4',
    'lf-cpp-8': 'LF8',
    'lf-cpp-16': 'LF16',
    'savina-akka-default': 'AK',
    'lf-c-1': 'LF C (1 thread)'
}
arrangementSequences = [
    'savina-akka-default',
    'lf-cpp-1',
    'lf-cpp-2',
    'lf-cpp-4',
    'lf-cpp-8',
    'lf-cpp-16',
    'lf-c-1',
]
cleanup = [
    "rm -rf build include lib share src-gen bin"
]

# parameters for the different benchmarks to run
numThreadsLfCpp1 = 1
numThreadsLfCpp2 = 2
numThreadsLfCpp4 = 4
numThreadsLfCpp8 = 8
numThreadsLfCpp16 = 16


experiments = {}


binName = 'PingPongBenchmark'
lfSrcPath = f'pingpong/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.pingpong.PingPongAkkaActorBenchmark'
runParamNameLf1 = f'--count'
runParamValue1 = '6000000'
runParamNameLf2 = f''
runParamValue2 = ''
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f''
preParamValue1 = ''
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -n {runParamValue1}'
experiments['PingPong'] = {
    'description': f'Benchmark Ping Pong from the Savina suite with {runParamValue1} pings.',
    'plotTitle': f'Ping Pong',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ runCmdLf4.split() ],
            'lf-cpp-8': [ runCmdLf8.split() ],
            'lf-cpp-16': [ runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 300'.split() ]
        })
    ]
}


binName = 'ThreadRingBenchmarkGenerator'
lfSrcPath = f'threadring/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.threadring.ThreadRingAkkaActorBenchmark'
runParamNameLf1 = f'--numPings'
runParamValue1 = '8000000'
runParamNameLf2 = f''
runParamValue2 = ''
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numReactors'
preParamValue1 = '100'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -n {preParamValue1} -r {runParamValue1}'
experiments['ThreadRing'] = {
    'description': f'Thread Ring from the Savina suite with {preParamValue1} actors/reactors.',
    'plotTitle': f'Thread Ring',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ runCmdLf4.split() ],
            'lf-cpp-8': [ runCmdLf8.split() ],
            'lf-cpp-16': [ runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}


binName = 'CountingBenchmark'
lfSrcPath = f'count/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.count.CountingAkkaActorBenchmark'
runParamNameLf1 = f'--countTo'
runParamValue1 = '20000000'
runParamNameLf2 = f''
runParamValue2 = ''
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f''
preParamValue1 = ''
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -n {runParamValue1}'
experiments['Counting'] = {
    'description': f'Counting benchmark from the Savina suite.',
    'plotTitle': f'Counting',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ runCmdLf4.split() ],
            'lf-cpp-8': [ runCmdLf8.split() ],
            'lf-cpp-16': [ runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}


binName = 'ThroughputBenchmarkGenerator'
lfSrcPath = f'fjthrput/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.fjthrput.ThroughputAkkaActorBenchmark'
runParamNameLf1 = f'--numMessagesPerReactor'
runParamValue1 = '100000'
runParamNameLf2 = f''
runParamValue2 = ''
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numWorkers'
preParamValue1 = '60'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -a {preParamValue1} -n {runParamValue1}'
experiments['Throughput'] = {
    'description': f'Fork join (throughput) benchmark from the Savina suite.',
    'plotTitle': f'Fork Join (throughput)',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ runCmdLf4.split() ],
            'lf-cpp-8': [ runCmdLf8.split() ],
            'lf-cpp-16': [ runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}


binName = 'ChameneosBenchmarkGenerator'
lfSrcPath = f'chameneos/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.chameneos.ChameneosAkkaActorBenchmark'
runParamNameLf1 = f'--numMeetings'
runParamValue1 = '500000'
runParamNameLf2 = f''
runParamValue2 = ''
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numChameneos'
preParamValue1 = '100'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -numChameneos {preParamValue1} -numMeetings {runParamValue1}'
experiments['Chameneos'] = {
    'description': f'Chameneos benchmark from the Savina suite.',
    'plotTitle': f'Chameneos',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ runCmdLf4.split() ],
            'lf-cpp-8': [ runCmdLf8.split() ],
            'lf-cpp-16': [ runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}



binName = 'BigBenchmarkGenerator'
lfSrcPath = f'big/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.big.BigAkkaActorBenchmark'
runParamNameLf1 = f'--numPingsPerReactor'
runParamValue1 = '20000'
runParamNameLf2 = f''
runParamValue2 = ''
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numReactors'
preParamValue1 = '15'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -w {preParamValue1} -n {runParamValue1}'
experiments['Big'] = {
    'description': f'Big benchmark from the Savina suite.',
    'plotTitle': f'Big',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('1', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         runCmdLf1.split() ],
            'lf-cpp-4': [ runCmdLf4.split() ],
            'lf-cpp-8': [ runCmdLf8.split() ],
            'lf-cpp-16': [ runCmdLf16.split() ],
            'savina-akka-default': [ f'{runCmdAkka}'.split() ]
        })
    ]
}




