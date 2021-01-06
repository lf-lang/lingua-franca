# This config is runs the concurrency benchmarks from Savina.
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


binName = 'DictionaryBenchmarkGenerator'
lfSrcPath = f'concdict/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.concdict.DictionaryAkkaActorBenchmark'
variableParamNameLF = f'--numMessagesPerWorker'
runParamNameLf1 = f''
runParamValue1 = ''
runParamNameLf2 = f'--writePercentage'
runParamValue2 = '10'
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
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -e {preParamValue1} -w {runParamValue2} -m '
experiments['Dictionary'] = {
    'description': f'Concurrent dictionary benchmark from the Savina suite.',
    'plotTitle': f'Concurrent dictionary',
    'plotXAxisLabel': 'Number of messages per worker in thousands',
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
                         f'{runCmdLf1} {variableParamNameLF} 10000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 10000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 10000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 10000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 10000'.split() ]
        }),
        ('20', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 20000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 20000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 20000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 20000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 20000'.split() ]
        }),
        ('30', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 30000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 30000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 30000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 30000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 30000'.split() ]
        }),
        ('40', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 40000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 40000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 40000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 40000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 40000'.split() ]
        })
    ]
}


binName = 'SortedListBenchmarkGenerator'
lfSrcPath = f'concsll/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.concsll.SortedListAkkaActorBenchmark'
variableParamNameLF = f'--numMessagesPerWorker'
runParamNameLf1 = f''
runParamValue1 = ''
runParamNameLf2 = f'--writePercentage'
runParamValue2 = '10'
runParamNameLf3 = f'--sizePercentage'
runParamValue3 = '1'
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
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -e {preParamValue1} -w {runParamValue2} -s {runParamValue3} -m '
experiments['SortedLinkedList'] = {
    'description': f'Sorted linked list benchmark from the Savina suite.',
    'plotTitle': f'Sorted linked list',
    'plotXAxisLabel': 'Number of messages per worker in thousands',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('4', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{runCmdLf1} {variableParamNameLF} 4000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 4000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 4000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 4000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 4000'.split() ]
        }),
        ('8', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 8000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 8000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 8000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 8000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 8000'.split() ]
        }),
        ('12', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 12000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 12000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 12000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 12000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 12000'.split() ]
        }),
        ('16', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 16000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 16000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 16000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 16000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 16000'.split() ]
        })
    ]
}


binName = 'ProdConsBenchmarkGenerator'
lfSrcPath = f'bndbuffer/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.bndbuffer.ProdConsAkkaActorBenchmark'
variableParamNameLF = f'--numItemsPerProducer'
runParamNameLf1 = f'--bufferSize'
runParamValue1 = '50'
runParamNameLf2 = f'--prodCost'
runParamValue2 = '25'
runParamNameLf3 = f'--consCost'
runParamValue3 = '25'
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numProducers'
preParamValue1 = '40'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f'numConsumers'
preParamValue2 = '40'
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -bb {runParamValue1} -np {preParamValue1} -nc {preParamValue2} -pc {runParamValue2} -cc {runParamValue3} -nm 1 -ipp '
experiments['ProducerConsumer'] = {
    'description': f'Producer-Consumer with Bounded Buffer benchmark from the Savina suite.',
    'plotTitle': f'Producer-Consumer',
    'plotXAxisLabel': 'Number of items produced per producer in thousands',
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
                         f'{runCmdLf1} {variableParamNameLF} 1000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 1000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 1000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 1000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 1000'.split() ]
        }),
        ('2', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 2000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 2000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 2000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 2000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 2000'.split() ]
        }),
        ('3', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 3000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 3000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 3000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 3000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 3000'.split() ]
        }),
        ('4', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 4000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 4000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 4000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 4000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 4000'.split() ]
        })
    ]
}



binName = 'PhilosopherBenchmarkGenerator'
lfSrcPath = f'philosopher/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.philosopher.PhilosopherAkkaActorBenchmark'
variableParamNameLF = f'--numEatingRounds'
runParamNameLf1 = f''
runParamValue1 = ''
runParamNameLf2 = f''
runParamValue2 = ''
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numPhilosophers'
preParamValue1 = '20'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -n {preParamValue1} -c 1 -m '
experiments['DiningPhilosophers'] = {
    'description': f'Dining Philosophers benchmark from the Savina suite.',
    'plotTitle': f'Dining Philosophers',
    'plotXAxisLabel': 'Number of eating rounds in thousands',
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
                         f'{runCmdLf1} {variableParamNameLF} 1000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 1000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 1000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 1000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 1000'.split() ]
        }),
        ('2', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 2000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 2000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 2000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 2000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 2000'.split() ]
        }),
        ('3', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 3000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 3000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 3000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 3000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 3000'.split() ]
        }),
        ('4', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 4000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 4000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 4000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 4000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 4000'.split() ]
        })
    ]
}



binName = 'SleepingBarberBenchmarkGenerator'
lfSrcPath = f'barber/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.barber.SleepingBarberAkkaActorBenchmark'
variableParamNameLF = f'numHaircuts'
runParamNameLf1 = f'--waitingRoomSize'
runParamValue1 = '1000'
runParamNameLf2 = f'--averageProductionRate'
runParamValue2 = '1000'
runParamNameLf3 = f'--averageHaircutRate'
runParamValue3 = '1000'
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
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -w {runParamValue1} -pr {runParamValue2} -hr {runParamValue3} -n '
experiments['SleepingBarber'] = {
    'description': f'Sleeping Barber benchmark from the Savina suite.',
    'plotTitle': f'Sleeping Barber',
    'plotXAxisLabel': 'Number of haircuts in hundreds',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('5', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r -D {variableParamNameLF}=500 {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{runCmdLf1}'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4}'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8}'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 500'.split() ]
        }),
        ('10', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r -D {variableParamNameLF}=1000 {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{runCmdLf1}'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4}'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8}'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 1000'.split() ]
        }),
        ('15', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r -D {variableParamNameLF}=1500 {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{runCmdLf1}'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4}'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8}'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 1500'.split() ]
        }),
        ('20', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r -D {variableParamNameLF}=2000 {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{runCmdLf1}'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4}'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8}'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16}'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 2000'.split() ]
        })
    ]
}


binName = 'CigaretteSmokerBenchmarkGenerator'
lfSrcPath = f'cigsmok/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.cigsmok.CigaretteSmokerAkkaActorBenchmark'
variableParamNameLF = f'--numRounds'
runParamNameLf1 = f''
runParamValue1 = ''
runParamNameLf2 = f''
runParamValue2 = ''
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numSmokers'
preParamValue1 = '200'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -s {preParamValue1} -r '
experiments['CigaretteSmokers'] = {
    'description': f'Cigarette Smokers benchmark from the Savina suite.',
    'plotTitle': f'Cigarette Smokers',
    'plotXAxisLabel': 'Number of rounds in thousands',
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
                         f'{runCmdLf1} {variableParamNameLF} 1000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 1000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 1000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 1000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 1000'.split() ]
        }),
        ('2', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 2000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 2000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 2000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 2000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 2000'.split() ]
        }),
        ('3', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 3000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 3000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 3000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 3000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 3000'.split() ]
        }),
        ('4', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 3000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 3000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 3000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 3000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 3000'.split() ]
        })
    ]
}


binName = 'LogisticMapBenchmarkGenerator'
lfSrcPath = f'logmap/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.logmap.LogisticMapAkkaManualStashActorBenchmark'
variableParamNameLF = f'--numTerms'
runParamNameLf1 = f''
runParamValue1 = ''
runParamNameLf2 = f'--startRate'
runParamValue2 = '3.46'
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numSeries'
preParamValue1 = '10'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -s {preParamValue1} -r {runParamValue2} -t '
experiments['LogisticMap'] = {
    'description': f'Logistic Map Series benchmark from the Savina suite.',
    'plotTitle': f'Logistic Map Series',
    'plotXAxisLabel': 'Number of terms in thousands',
    'plotYAxisLabel': 'Execution time in ms (median)',
    'plotSequenceColors': sequenceColors,
    'plotSequenceNames': sequenceNames,
    'plotArrangementSequences': arrangementSequences,
    'parsers': parsers,
    'finalCleanupExperiment': cleanup,
    'sequences': [
        ('25', {
            'lf-cpp-1': [ f'{pythonExe} -m cogapp -r {preParamString1} {preParamString2} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{lfcExe} {os.path.join(lfCppSourceFilePathBase, lfSrcPath)}'.split(),
                         f'{runCmdLf1} {variableParamNameLF} 25000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 25000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 25000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 25000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 25000'.split() ]
        }),
        ('50', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 50000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 50000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 50000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 50000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 50000'.split() ]
        }),
        ('75', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 75000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 75000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 75000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 75000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 75000'.split() ]
        }),
        ('100', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 100000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 100000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 100000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 100000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 100000'.split() ]
        })
    ]
}


binName = 'BankingBenchmarkGenerator'
lfSrcPath = f'banking/{binName}.lf'
akkaPkgPath = f'{savinaPackagePathBase}.banking.BankingAkkaManualStashActorBenchmark'
variableParamNameLF = f'--numTransactions'
runParamNameLf1 = f''
runParamValue1 = ''
runParamNameLf2 = f''
runParamValue2 = ''
runParamNameLf3 = f''
runParamValue3 = ''
runParamNameLf4 = f''
runParamValue4 = ''
preParamNameLf1 = f'numAccounts'
preParamValue1 = '20'
preParamString1 = f'-D {preParamNameLf1}={preParamValue1}' if len(preParamNameLf1) > 0 else ''
preParamNameLf2 = f''
preParamValue2 = ''
preParamString2 = f'-D {preParamNameLf2}={preParamValue2}' if len(preParamNameLf2) > 0 else ''
runCmdLf1 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp1} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf4 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp4} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf8 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp8} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdLf16 = f'bin/{binName} --fast --numIterations {numIterationsDefault} --threads {numThreadsLfCpp16} {runParamNameLf1} {runParamValue1} {runParamNameLf2} {runParamValue2} {runParamNameLf3} {runParamValue3} {runParamNameLf4} {runParamValue4}'
runCmdAkka = f'{javaExe} -classpath {savinaJarPath} {akkaPkgPath} -iter {numIterationsAkka} -a {preParamValue1} -n '
experiments['BankingTransaction'] = {
    'description': f'Bank Transaction benchmark from the Savina suite.',
    'plotTitle': f'Bank Transaction',
    'plotXAxisLabel': 'Number of transactions in thousands',
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
                         f'{runCmdLf1} {variableParamNameLF} 10000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 10000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 10000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 10000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 10000'.split() ]
        }),
        ('20', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 20000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 20000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 20000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 20000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 20000'.split() ]
        }),
        ('30', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 30000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 30000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 30000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 30000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 30000'.split() ]
        }),
        ('40', {
            'lf-cpp-1': [ f'{runCmdLf1} {variableParamNameLF} 40000'.split() ],
            'lf-cpp-4': [ f'{runCmdLf4} {variableParamNameLF} 40000'.split() ],
            'lf-cpp-8': [ f'{runCmdLf8} {variableParamNameLF} 40000'.split() ],
            'lf-cpp-16': [ f'{runCmdLf16} {variableParamNameLF} 40000'.split() ],
            'savina-akka-default': [ f'{runCmdAkka} 40000'.split() ]
        })
    ]
}
