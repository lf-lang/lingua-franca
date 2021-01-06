# This config is runs the parallelism benchmarks from Savina.
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


binName = 'ApspBenchmarkGenerator'
lfSrcPath = f'apsp/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.apsp.ApspAkkaActorBenchmark'
runParamNameLf1 = f'--maxEdgeWeight'
runParamValue1 = '100'
runParamNameLf2 = f''
runParamValue2 = ''
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numNodes'
preParamValue1 = '300'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f'blockSize'
preParamValue2 = '50'
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -n {preParamValue1} -b {preParamValue2} -w {runParamValue1}'
experiments['Apsp'] = {
    'description': f'All Pairs Shortest Path benchmark from the Savina suite.',
    'plotTitle': f'All Pairs Shortest Path',
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


binName = 'GuidedSearchBenchmarkGenerator'
lfSrcPath = f'astar/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.astar.GuidedSearchAkkaActorBenchmark'
runParamNameLf1 = f'--threshold'
runParamValue1 = '1024'
runParamNameLf2 = f'--gridSize'
runParamValue2 = '30'
runParamNameLf3 = f'--priorities'
runParamValue3 = '30'
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numWorkers'
preParamValue1 = '20'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -w {preParamValue1} -t {runParamValue1} -g {runParamValue2} -p {runParamValue3}'
experiments['Astar'] = {
    'description': f'A-star benchmark from the Savina suite.',
    'plotTitle': f'A-star',
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


binName = 'NQueensBenchmarkGenerator'
lfSrcPath = f'nqueenk/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.nqueenk.NQueensAkkaActorBenchmark'
runParamNameLf1 = f'--size'
runParamValue1 = '12'
runParamNameLf2 = f'--threshold'
runParamValue2 = '4'
runParamNameLf3 = f'--solutionsLimit'
runParamValue3 = '1500000'
runParamNameLf4 = f'--priorities'
runParamValue4 = '10'
preParamNameLf1 = f'numWorkers'
preParamValue1 = '20'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -n {runParamValue1} -t {runParamValue2} -w {preParamValue1} -s {runParamValue3} -p {runParamValue4}'
experiments['NQueens'] = {
    'description': f'N Queens k Solutions benchmark from the Savina suite.',
    'plotTitle': f'N Queens k Solutions',
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


binName = 'MatMulBenchmarkGenerator'
lfSrcPath = f'recmatmul/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.recmatmul.MatMulAkkaActorBenchmark'
runParamNameLf1 = f'--dataLength'
runParamValue1 = '1024'
runParamNameLf2 = f'--blockThreshold'
runParamValue2 = '16384'
runParamNameLf3 = f'--priorities'
runParamValue3 = '10'
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numWorkers'
preParamValue1 = '20'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -n {runParamValue1} -t {runParamValue2} -w {preParamValue1} -p {runParamValue3}'
experiments['MatrixMultiplication'] = {
    'description': f'Recursive Matrix Multiplication benchmark from the Savina suite.',
    'plotTitle': f'Recursive Matrix Multiplication',
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


binName = 'RadixSortBenchmark'
lfSrcPath = f'radixsort/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.radixsort.RadixSortAkkaActorBenchmark'
runParamNameLf1 = f'--dataSize'
runParamValue1 = '100000'
runParamNameLf2 = f'--maxValue'
runParamValue2 = '1152921504606846976'
runParamNameLf3 = f'--seed'
runParamValue3 = '2048'
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
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -n {runParamValue1} -m {runParamValue2} -s {runParamValue3}'
experiments['RadixSort'] = {
    'description': f'Radix Sort benchmark from the Savina suite.',
    'plotTitle': f'Radix Sort',
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


binName = 'FilterBankBenchmarkGenerator'
lfSrcPath = f'filterbank/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.filterbank.FilterBankAkkaActorBenchmark'
runParamNameLf1 = f'--numSimulations'
runParamValue1 = '34816'
runParamNameLf2 = f'--numColumns'
runParamValue2 = '16384'
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numChannels'
preParamValue1 = '8'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -sim {runParamValue1} -col {runParamValue2} -chan {preParamValue1}'
experiments['FilterBank'] = {
    'description': f'Filter Bank benchmark from the Savina suite.',
    'plotTitle': f'Filter Bank',
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


binName = 'TrapezoidalBenchmarkGenerator'
lfSrcPath = f'trapezoid/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.trapezoid.TrapezoidalAkkaActorBenchmark'
runParamNameLf1 = f'--numPieces'
runParamValue1 = '10000000'
runParamNameLf2 = f'--leftEndPoint'
runParamValue2 = '1.0'
runParamNameLf3 = f'--rightEndPoint'
runParamValue3 = '5.0'
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numWorkers'
preParamValue1 = '100'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -n {runParamValue1} -w {preParamValue1} -l {runParamValue2} -r {runParamValue3}'
experiments['TrapezoidalApproximation'] = {
    'description': f'Trapezoidal Approximation benchmark from the Savina suite.',
    'plotTitle': f'Trapezoidal Approximation',
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


binName = 'PiPrecisionBenchmarkGenerator'
lfSrcPath = f'piprecision/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.piprecision.PiPrecisionAkkaActorBenchmark'
runParamNameLf1 = f'--precision'
runParamValue1 = '5000'
runParamNameLf2 = f''
runParamValue2 = ''
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numWorkers'
preParamValue1 = '20'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -w {preParamValue1} -p {runParamValue1}'
experiments['PiPrecision'] = {
    'description': f'Precise Pi Computation benchmark from the Savina suite.',
    'plotTitle': f'Precise Pi Computation',
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



