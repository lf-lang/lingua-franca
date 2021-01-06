# This config is runs the parallelism benchmarks from Savina.
#
# It runs the Akka implementation and LF C++ implementations.
#
# Recommended plotter module: plotterDefault

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
variableParamNameLF = f'numNodes'
runParamNameLf1 = f'--maxEdgeWeight'
runParamValue1 = '100'
runParamNameLf2 = f''
runParamValue2 = ''
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f''
preParamValue1 = ''
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f'blockSize'
preParamValue2 = '50'
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -b {preParamValue2} -w {runParamValue1} -n '
experiments['Apsp'] = {
    'description': f'All Pairs Shortest Path benchmark from the Savina suite.',
    'plotTitle': f'All Pairs Shortest Path',
    'plotXAxisLabel': 'Number of nodes in hundreds',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('3', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r -D {variableParamNameLF}=300 {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{runCmdLf1}'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4}'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8}'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 300'.split() ]
        }),
        ('4', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r -D {variableParamNameLF}=400 {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{runCmdLf1}'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4}'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8}'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 400'.split() ]
        }),
        ('5', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r -D {variableParamNameLF}=500 {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{runCmdLf1}'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4}'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8}'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 500'.split() ]
        }),
        ('6', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r -D {variableParamNameLF}=600 {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{runCmdLf1}'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4}'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8}'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 600'.split() ]
        })
    ]
}


binName = 'GuidedSearchBenchmarkGenerator'
lfSrcPath = f'astar/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.astar.GuidedSearchAkkaActorBenchmark'
variableParamNameLF = f'--gridSize'
runParamNameLf1 = f'--threshold'
runParamValue1 = '1024'
runParamNameLf2 = f''
runParamValue2 = ''
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
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -w {preParamValue1} -t {runParamValue1} -p {runParamValue3} -g '
experiments['Astar'] = {
    'description': f'A-star benchmark from the Savina suite.',
    'plotTitle': f'A-star',
    'plotXAxisLabel': 'Grid size',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('30', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{runCmdLf1} {variableParamNameLF} 30'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 30'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 30'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 30'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 30'.split() ]
        }),
        ('60', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 60'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 60'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 60'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 60'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 60'.split() ]
        }),
        ('90', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 90'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 90'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 90'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 90'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 90'.split() ]
        }),
        ('120', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 120'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 120'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 120'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 120'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 120'.split() ]
        })
    ]
}


binName = 'NQueensBenchmarkGenerator'
lfSrcPath = f'nqueenk/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.nqueenk.NQueensAkkaActorBenchmark'
variableParamNameLF = f'--size'
runParamNameLf1 = f''
runParamValue1 = ''
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
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -t {runParamValue2} -w {preParamValue1} -s {runParamValue3} -p {runParamValue4} -n '
experiments['NQueens'] = {
    'description': f'N Queens k Solutions benchmark from the Savina suite.',
    'plotTitle': f'N Queens k Solutions',
    'plotXAxisLabel': 'Size',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('12', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{runCmdLf1} {variableParamNameLF} 12'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 12'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 12'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 12'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 12'.split() ]
        }),
        ('13', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 13'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 13'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 13'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 13'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 13'.split() ]
        }),
        ('14', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 14'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 14'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 14'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 14'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 14'.split() ]
        }),
        ('15', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 15'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 15'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 15'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 15'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 15'.split() ]
        })
    ]
}


binName = 'MatMulBenchmarkGenerator'
lfSrcPath = f'recmatmul/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.recmatmul.MatMulAkkaActorBenchmark'
variableParamNameLF = f'--dataLength'
runParamNameLf1 = f''
runParamValue1 = ''
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
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -t {runParamValue2} -w {preParamValue1} -p {runParamValue3} -n '
experiments['MatrixMultiplication'] = {
    'description': f'Recursive Matrix Multiplication benchmark from the Savina suite.',
    'plotTitle': f'Recursive Matrix Multiplication',
    'plotXAxisLabel': 'Number of rows',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('256', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{runCmdLf1} {variableParamNameLF} 256'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 256'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 256'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 256'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 256'.split() ]
        }),
        ('512', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 512'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 512'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 512'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 512'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 512'.split() ]
        }),
        ('1024', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 1024'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 1024'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 1024'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 1024'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 1024'.split() ]
        }),
        ('2048', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 2048'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 2048'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 2048'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 2048'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 2048'.split() ]
        })
    ]
}


binName = 'RadixSortBenchmark'
lfSrcPath = f'radixsort/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.radixsort.RadixSortAkkaActorBenchmark'
variableParamNameLF = f'--dataSize'
runParamNameLf1 = f''
runParamValue1 = ''
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
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -m {runParamValue2} -s {runParamValue3} -n '
experiments['RadixSort'] = {
    'description': f'Radix Sort benchmark from the Savina suite.',
    'plotTitle': f'Radix Sort',
    'plotXAxisLabel': 'Number of data elements to sort in thousands',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('100', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{runCmdLf1} {variableParamNameLF} 100000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 100000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 100000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 100000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 100000'.split() ]
        }),
        ('200', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 200000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 200000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 200000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 200000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 200000'.split() ]
        }),
        ('300', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 300000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 300000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 300000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 300000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 300000'.split() ]
        }),
        ('400', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 400000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 400000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 400000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 400000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 400000'.split() ]
        })
    ]
}


binName = 'FilterBankBenchmarkGenerator'
lfSrcPath = f'filterbank/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.filterbank.FilterBankAkkaActorBenchmark'
variableParamNameLF = f'--numSimulations'
runParamNameLf1 = f''
runParamValue1 = ''
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
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -col {runParamValue2} -chan {preParamValue1} -sim '
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
        ('16384', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{runCmdLf1} {variableParamNameLF} 16384'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 16384'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 16384'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 16384'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 16384'.split() ]
        }),
        ('20480', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 20480'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 20480'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 20480'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 20480'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 20480'.split() ]
        }),
        ('24576', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 24576'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 24576'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 24576'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 24576'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 24576'.split() ]
        }),
        ('28672', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 28672'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 28672'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 28672'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 28672'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 28672'.split() ]
        })
    ]
}


binName = 'TrapezoidalBenchmarkGenerator'
lfSrcPath = f'trapezoid/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.trapezoid.TrapezoidalAkkaActorBenchmark'
variableParamNameLF = f'--numPieces'
runParamNameLf1 = f''
runParamValue1 = ''
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
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -w {preParamValue1} -l {runParamValue2} -r {runParamValue3} -n '
experiments['TrapezoidalApproximation'] = {
    'description': f'Trapezoidal Approximation benchmark from the Savina suite.',
    'plotTitle': f'Trapezoidal Approximation',
    'plotXAxisLabel': 'Number of pieces in millions',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('10', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{runCmdLf1} {variableParamNameLF} 10000000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 10000000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 10000000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 10000000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 10000000'.split() ]
        }),
        ('20', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 20000000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 20000000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 20000000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 20000000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 20000000'.split() ]
        }),
        ('30', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 30000000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 30000000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 30000000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 30000000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 30000000'.split() ]
        }),
        ('40', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 40000000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 40000000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 40000000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 40000000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 40000000'.split() ]
        })
    ]
}


binName = 'PiPrecisionBenchmarkGenerator'
lfSrcPath = f'piprecision/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.piprecision.PiPrecisionAkkaActorBenchmark'
variableParamNameLF = f'--precision'
runParamNameLf1 = f''
runParamValue1 = ''
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
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -w {preParamValue1} -p '
experiments['PiPrecision'] = {
    'description': f'Precise Pi Computation benchmark from the Savina suite.',
    'plotTitle': f'Precise Pi Computation',
    'plotXAxisLabel': 'Precision in thousand digits',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('5', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{runCmdLf1} {variableParamNameLF} 5000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 5000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 5000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 5000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 5000'.split() ]
        }),
        ('7', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 7000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 7000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 7000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 7000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 7000'.split() ]
        }),
        ('9', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 9000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 9000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 9000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 9000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 9000'.split() ]
        }),
        ('11', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 11000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 11000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 11000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 11000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 11000'.split() ]
        })
    ]
}



